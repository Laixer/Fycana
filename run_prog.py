#!/usr/bin/env python3

import os
import sys
import csv
import time
import json
import time
import traceback
import numpy as np
import argparse
import configparser

from datetime import datetime

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import format_euler_tuple, format_angle_both
from pyglonax.alg import shortest_rotation


class Executor:
    def __init__(self, definition_file, host, supervisor=True, trace=False, **kwargs):
        self.excavator = Excavator.from_json(file_path=definition_file)
        self.adapter = ExcavatorAdapter(host=host)
        self.supervisor = supervisor
        self.trace = trace
        self.adapter.on_signal_update(self._update_signal)
        self.tolerance = float(kwargs.get("tolerance", 0.01))
        self.articulation_chain = self.excavator.get_chain_by_name("articulation_arm")

    def _update_signal(self, signal):
        for joint in self.articulation_chain.joints:
            if joint.name in self.adapter.encoder:
                self.excavator.set_position_state(
                    joint.name, self.adapter.encoder[joint.name]["angle"]
                )

    def solve_target(self, step, target):
        effector = self.articulation_chain.opspace_forward_kinematics()

        print("")
        print(f"Step {step} projection:")
        print("Target        : ", format_euler_tuple(target))
        print("Effector      : ", format_euler_tuple(effector))
        print("Opspace Error : ", format_euler_tuple(target - effector))

        self.articulation_chain.inverse_kinematics(target[:3])

        forward_projection = self.articulation_chain.projected_forward_kinematics()
        print("  IK: Projected Pitch:", format_angle_both(forward_projection[4]))
        abs_error = target[4] - forward_projection[4]
        print("  IK: AbsPitch error:", format_angle_both(abs_error))
        rel_pitch_error0 = self.excavator.attachment + abs_error
        print("  IK: RelPitch error:", format_angle_both(rel_pitch_error0))

        is_within_bounds = self.excavator.attachment_joint.is_within_bounds(
            rel_pitch_error0
        )
        print("  IK: Is Within bounds:", is_within_bounds)
        if not is_within_bounds:
            rel_pitch_error0 = self.excavator.attachment_joint.clip(rel_pitch_error0)
            print("  IK: Clipped angle:", rel_pitch_error0)

        print()
        if self.supervisor:
            input("Press Enter to start...")

        trace_file = None
        trace_writer = None
        if self.trace:
            unix_time = int(time.time())

            trace_file = open(f"trace/prog_{unix_time}_step_{step}.csv", "w")
            trace_writer = csv.writer(trace_file)

            header = [
                "iteration",
                "timestamp",
                "frame_error",
                "frame_power",
                "boom_error",
                "boom_power",
                "arm_error",
                "arm_power",
                "attachment_error",
                "attachment_power",
            ]
            trace_writer.writerow(header)

        count = 0
        while True:
            print()
            print(f"Step: {step} Iter: {count} Target:", format_euler_tuple(target))

            self.articulation_chain.position_state[3] = rel_pitch_error0

            rel_error = self.articulation_chain.error()
            rel_power = np.zeros(rel_error.shape[0], dtype=np.int32)

            rel_error[0] = shortest_rotation(rel_error[0])

            motion_list = []

            for idx, joint in enumerate(self.articulation_chain.joints):
                rel_power[idx] = joint.motion_profile.power(rel_error[idx])

                print(
                    "{:<15}".format(joint.name),
                    "  Error: {:>15}".format(format_angle_both(rel_error[idx])),
                    "  Power: {:>6d}".format(rel_power[idx]),
                )

                actuator = None
                if joint.name == "frame":
                    actuator = ExcavatorActuator.Slew
                elif joint.name == "boom":
                    actuator = ExcavatorActuator.Boom
                elif joint.name == "arm":
                    actuator = ExcavatorActuator.Arm
                elif joint.name == "attachment":
                    actuator = ExcavatorActuator.Attachment

                motion_list.append((actuator, rel_power[idx]))

            self.adapter.change(motion_list)

            if trace_writer is not None:
                now = datetime.now()

                data = [
                    count,
                    now.isoformat(),
                    rel_error[0],
                    rel_power[0],
                    rel_error[1],
                    rel_power[1],
                    rel_error[2],
                    rel_power[2],
                    rel_error[3],
                    rel_power[3],
                ]
                trace_writer.writerow(data)

            if np.all(np.abs(rel_error) < self.tolerance):
                if trace_file is not None:
                    trace_file.close()

                print()
                print("Objective reached")
                if self.supervisor:
                    time.sleep(1)

                effector = self.articulation_chain.opspace_forward_kinematics()
                print("Target        : ", format_euler_tuple(target))
                print("Effector      : ", format_euler_tuple(effector))
                print("Opspace Error : ", format_euler_tuple(target - effector))
                if self.supervisor:
                    input("Press Enter to continue...")
                break

            count += 1
            time.sleep(0.05)

    def start(self):
        print(self.excavator)

        self.adapter.start()
        self.adapter.wait_until_initialized()
        self.adapter.enable_motion()

        print("Machine initialized")
        print("Kinematic tolerance:", self.tolerance)

    def stop(self):
        self.adapter.disable_motion()
        self.adapter.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("program", help="program file")
    parser.add_argument("-n", "--no-supervisor", action="store_false")
    parser.add_argument("-t", "--trace", action="store_true")
    parser.add_argument("-c", "--config", default="config.ini", help="config file")

    args = parser.parse_args()

    config = configparser.ConfigParser()
    config.read(args.config)

    host = config["glonax"]["host"]
    port = config["glonax"]["port"]

    f = open(args.program)
    json_file = json.load(f)
    program = np.array(json_file)
    f.close()

    robot = dict(config["robot"])
    kinematics = dict(config["kinematics"])

    executor = Executor(
        host=f"{host}:{port}",
        supervisor=args.no_supervisor,
        trace=args.trace,
        **robot,
        **kinematics,
    )
    try:
        executor.start()

        print()
        print("Program:")
        for idx, target in enumerate(program):
            print(f"{idx}", format_euler_tuple(target))

        for idx, target in enumerate(program):
            executor.solve_target(idx, target)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
    finally:
        executor.stop()

# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument("-n", "--no-supervisor", action="store_true")
#     parser.add_argument("-t", "--trace", action="store_true")
#     parser.add_argument("-c", "--config", default="config.ini", help="config file")

#     args = parser.parse_args()

#     config = configparser.ConfigParser()
#     config.read(args.config)

#     host = config["glonax"]["host"]
#     port = config["glonax"]["port"]

#     robot = dict(config["robot"])
#     kinematics = dict(config["kinematics"])

#     executor = Executor(
#         host=f"{host}:{port}",
#         supervisor=True,
#         trace=args.trace,
#         **robot,
#         **kinematics,
#     )
#     try:
#         import random

#         executor.start()

#         idx = 0
#         while True:
#             random_frame = random.randint(0, 6283) / 1000
#             random_boom = random.randint(-1047, 785) / 1000
#             random_arm = random.randint(680, 2760) / 1000
#             random_attachment = random.randint(-962, 2190) / 1000

#             x = [0, random_frame, random_boom, random_arm, random_attachment, 0]
#             # x = [0, 6.171, -1.044, 1.071, -0.087, 0]
#             print("Random X:", x)
#             target = executor.excavator._calculate_forward_kinematics(
#                 x, joint_name="attachment_joint"
#             )[-1]

#             # target = np.array([4.00, -1.29, 2.7, 0.0, 0.0, 0])

#             for joint in self.articulation_chain.joints:
#                 if joint.name in self.adapter.encoder:
#                     self.excavator.set_position_state(
#                         joint.name, self.adapter.encoder[joint.name]["angle"]
#                     )

#             executor.solve_target(idx, target)
#             idx += 1
#     except KeyboardInterrupt:
#         pass
#     except Exception as e:
#         traceback.print_exception(type(e), e, e.__traceback__)
#     finally:
#         executor.stop()
