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
        self.excavator = Excavator.from_urdf(file_path=definition_file)
        self.adapter = ExcavatorAdapter(host=host)
        self.supervisor = supervisor
        self.trace = trace
        self.adapter.on_signal_update(self._update_signal)
        self.tolerance = float(kwargs.get("tolerance", 0.01))

    def _update_signal(self, signal):
        if "frame" in self.adapter.encoder:
            self.excavator.frame = self.adapter.encoder["frame"]["angle"]
        if "boom" in self.adapter.encoder:
            self.excavator.boom = self.adapter.encoder["boom"]["angle"]
        if "arm" in self.adapter.encoder:
            self.excavator.arm = self.adapter.encoder["arm"]["angle"]
        if "attachment" in self.adapter.encoder:
            self.excavator.attachment = self.adapter.encoder["attachment"]["angle"]

    def solve_target(self, idx, target):
        effector = self.excavator.forward_kinematics2(joint_name="attachment_joint")

        print("")
        print(f"Step {idx} projection:")
        print("Target   :", format_euler_tuple(target))
        print("Effector :", format_euler_tuple(effector))

        self.excavator.inverse_kinematics(target[:3])

        proj_angle = np.sum(self.excavator.get_position_state_projected()[2:])
        print("IK: Projected Pitch:", format_angle_both(proj_angle))
        abs_error = target[4] - proj_angle
        print("IK: AbsPitch error:", format_angle_both(abs_error))
        rel_pitch_error0 = self.excavator.attachment + abs_error
        print("IK: RelPitch error:", format_angle_both(rel_pitch_error0))

        bc = self.excavator.attachment_joint.is_within_bounds(rel_pitch_error0)
        print("IK: Is Within bounds:", bc)
        if not bc:
            rel_pitch_error0 = self.excavator.attachment_joint.clip(rel_pitch_error0)
            print("IK: Clipped angle:", rel_pitch_error0)

        self.excavator.position_state[1][4] = rel_pitch_error0

        print()
        if self.supervisor:
            input("Press Enter to start...")

        trace_file = None
        trace_writer = None
        if self.trace:
            unix_time = int(time.time())

            trace_file = open(f"trace/prog_{unix_time}_step_{idx}.csv", "w")
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
            print(f"Step: {idx} Iter: {count} Target:", format_euler_tuple(target))

            rel_error = self.excavator.get_position_error()[0]

            rel_frame_error = shortest_rotation(rel_error[1])
            rel_boom_error = rel_error[2]
            rel_arm_error = rel_error[3]
            rel_attachment_error = rel_error[4]

            power_setting_slew = (
                self.excavator.frame_joint.motion_profile.proportional_power(
                    rel_frame_error
                )
            )
            power_setting_boom = (
                self.excavator.boom_joint.motion_profile.proportional_power(
                    rel_boom_error
                )
            )
            power_setting_arm = (
                self.excavator.arm_joint.motion_profile.proportional_power_inverse(
                    rel_arm_error
                )
            )
            power_setting = (
                self.excavator.attachment_joint.motion_profile.proportional_power(
                    rel_attachment_error
                )
            )

            print(
                "{:<15}".format("Frame"),
                "  Error: {:>15}".format(format_angle_both(rel_frame_error)),
                "  Power: {:>6d}".format(int(power_setting_slew)),
            )
            print(
                "{:<15}".format("Boom"),
                "  Error: {:>15}".format(format_angle_both(rel_boom_error)),
                "  Power: {:>6d}".format(int(power_setting_boom)),
            )
            print(
                "{:<15}".format("Arm"),
                "  Error: {:>15}".format(format_angle_both(rel_arm_error)),
                "  Power: {:>6d}".format(int(power_setting_arm)),
            )
            print(
                "{:<15}".format("Attachment"),
                "  Error: {:>15}".format(format_angle_both(rel_attachment_error)),
                "  Power: {:>6d}".format(int(power_setting)),
            )

            self.adapter.change(
                [
                    (ExcavatorActuator.Slew, int(power_setting_slew)),
                    (ExcavatorActuator.Boom, int(power_setting_boom)),
                    (ExcavatorActuator.Arm, int(power_setting_arm)),
                    (ExcavatorActuator.Attachment, int(power_setting)),
                ]
            )

            if trace_writer is not None:
                now = datetime.now()

                data = [
                    count,
                    now.isoformat(),
                    rel_frame_error,
                    power_setting_slew,
                    rel_boom_error,
                    power_setting_boom,
                    rel_arm_error,
                    power_setting_arm,
                    rel_attachment_error,
                    power_setting,
                ]
                trace_writer.writerow(data)

            if (
                abs(rel_frame_error) < self.tolerance
                and abs(rel_boom_error) < self.tolerance
                and abs(rel_arm_error) < self.tolerance
                and abs(rel_attachment_error) < self.tolerance
            ):
                if trace_file is not None:
                    trace_file.close()

                print()
                print("Objective reached")
                if self.supervisor:
                    time.sleep(1)
                effector = self.excavator.forward_kinematics2(
                    joint_name="attachment_joint"
                )
                print("Target   :", format_euler_tuple(target))
                print("Effector :", format_euler_tuple(effector))
                print("EmpError :", format_euler_tuple(target - effector))
                if self.supervisor:
                    input("Press Enter to continue...")
                break

            count += 1
            time.sleep(0.05)

    def start(self):
        self.adapter.start()
        self.adapter.wait_until_initialized()
        self.adapter.enable_motion()

        print("Kinematic tolerance:", self.tolerance)
        print("Motion profile slew:", 10_000, 12_000)
        print("Motion profile boom:", 15_000, 12_000)
        print("Motion profile arm:", 15_000, 12_000)
        print("Motion profile attachment:", 15_000, 12_000)

    def stop(self):
        self.adapter.disable_motion()
        self.adapter.stop()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("program", help="program file")
    parser.add_argument("-n", "--no-supervisor", action="store_true")
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
        supervisor=~args.no_supervisor,
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

#             executor.solve_target(idx, target)
#             idx += 1
#     except KeyboardInterrupt:
#         pass
#     except Exception as e:
#         traceback.print_exception(type(e), e, e.__traceback__)
#     finally:
#         executor.stop()
