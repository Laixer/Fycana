#!/usr/bin/env python3

import os
import sys
import time
import json
import time
import traceback
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import get_config, format_euler_tuple, format_angle_both
from pyglonax.alg import shortest_rotation

config = get_config()

np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})


class MotionProfile:
    def __init__(self, scale, offset, lower_bound, inverse):
        self.scale = scale
        self.offset = offset
        self.lower_bound = lower_bound
        self.inverse = inverse

    def proportional_power(self, value):
        if abs(value) > self.lower_bound:
            power = self.offset + min((abs(value) * self.scale), 32_767 - self.offset)
            if value < 0:
                return -power
            else:
                return power
        else:
            return 0

    def proportional_power_inverse(self, value):
        if abs(value) > self.lower_bound:
            power = value * self.scale

            if value > 0:
                return max(-power, -(32_767 - self.offset)) - self.offset
            else:
                return min(-power, 32_767 - self.offset) + self.offset
        else:
            return 0


tolerance = float(config["ROBOT_KIN_TOL"])

motion_profile_slew = MotionProfile(10_000, 12_000, tolerance, False)
motion_profile_boom = MotionProfile(15_000, 12_000, tolerance, True)
motion_profile_arm = MotionProfile(15_000, 12_000, tolerance, False)
motion_profile_attachment = MotionProfile(15_000, 12_000, tolerance, False)


class Executor:
    def __init__(self, definition_file, host, supervisor=True):
        self.excavator = Excavator.from_urdf(file_path=definition_file)
        self.adapter = ExcavatorAdapter(host=host)
        self.supervisor = supervisor
        self.adapter.on_signal_update(self._update_signal)

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
            input("Press Enter to continue...")

        count = 0
        while True:
            count += 1

            print()
            print(f"Iter: {count} Target:", format_euler_tuple(target))

            rel_error = self.excavator.get_position_error()[0]

            rel_frame_error = shortest_rotation(rel_error[1])
            rel_boom_error = rel_error[2]
            rel_arm_error = rel_error[3]
            rel_attachment_error = rel_error[4]

            power_setting_slew = motion_profile_slew.proportional_power(rel_frame_error)
            power_setting_boom = motion_profile_boom.proportional_power(rel_boom_error)
            power_setting_arm = motion_profile_arm.proportional_power_inverse(
                rel_arm_error
            )
            power_setting = motion_profile_attachment.proportional_power(
                rel_attachment_error
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

            if (
                abs(rel_frame_error) < tolerance
                and abs(rel_boom_error) < tolerance
                and abs(rel_arm_error) < tolerance
                and abs(rel_attachment_error) < tolerance
            ):
                print()
                print("Objective reached")
                if self.supervisor:
                    time.sleep(0.75)
                effector = self.excavator.forward_kinematics2(
                    joint_name="attachment_joint"
                )
                print("Target   :", format_euler_tuple(target))
                print("Effector :", format_euler_tuple(effector))
                if self.supervisor:
                    input("Press Enter to continue...")
                break

            time.sleep(0.05)

    def start(self, program):
        self.adapter.start()
        self.adapter.wait_until_initialized()
        self.adapter.enable_motion()

        print("Program:")
        for idx, target in enumerate(program):
            print(f"{idx}", format_euler_tuple(target))

        for idx, target in enumerate(program):
            self.solve_target(idx, target)

    def stop(self):
        self.adapter.disable_motion()
        self.adapter.stop()


if __name__ == "__main__":
    args = sys.argv[1:]

    if len(args) < 1:
        print("Usage: python3 run_prog.py <program.json> [--no-supervisor]")
        sys.exit(1)

    print("Loading model:", args[0])

    json_file = json.load(open(f"{args[0]}"))
    program = np.array(json_file)

    supervisor = True
    if len(args) == 2:
        if args[1] == "--no-supervisor":
            supervisor = False

    executor = Executor(
        definition_file=config["ROBOT_DEFINITION"],
        host=config["GLONAX_HOST"],
        supervisor=supervisor,
    )
    try:
        executor.start(program)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
    finally:
        executor.stop()
