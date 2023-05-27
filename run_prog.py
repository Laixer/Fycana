#!/usr/bin/env python3

import os
import sys
import time
import json
import time
import traceback
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import get_config
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
            power = self.offset + \
                min((abs(value) * self.scale), 32_767 - self.offset)
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


def format_angle(value):
    return "{:.2f}°".format(np.rad2deg(value))


def format_angle_both(value):
    return "{:.3f}  {:.2f}°".format(value, np.rad2deg(value))


def format_coord(value):
    return "{:.2f}".format(value)


def format_euler_tuple(effector):
    return "({}, {}, {}) [{}, {}, {}]".format(
        format_coord(effector[0]),
        format_coord(effector[1]),
        format_coord(effector[2]),
        format_angle(effector[3]),
        format_angle(effector[4]),
        format_angle(effector[5]),
    )


tolerance = float(config["ROBOT_KIN_TOL"])

motion_profile_slew = MotionProfile(10_000, 12_000, tolerance, False)
motion_profile_boom = MotionProfile(15_000, 12_000, tolerance, True)
motion_profile_arm = MotionProfile(15_000, 12_000, tolerance, False)
motion_profile_attachment = MotionProfile(15_000, 12_000, tolerance, False)

excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])
adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])

args = sys.argv[1:]

if len(args) < 1:
    print("Usage: python3 run_prog.py <program.json>")
    sys.exit(1)

adapter.start()
adapter.wait_until_initialized()

print("Machine initialized")

excavator.frame = adapter.encoder["frame"]["angle"]
excavator.boom = adapter.encoder["boom"]["angle"]
excavator.arm = adapter.encoder["arm"]["angle"]
excavator.attachment = adapter.encoder["attachment"]["angle"]

print("Loading model:", args[0])

json_file = json.load(open(f"{args[0]}"))
program = np.array(json_file)

print("Program:")
for idx, target in enumerate(program):
    print(f"{idx}", format_euler_tuple(target))

SUPERVISOR = True

if len(args) == 2:
    if args[1] == "--no-supervisor":
        SUPERVISOR = False

print()
print("Starting program")
if SUPERVISOR:
    input("Press Enter to continue...")


def move_to_target(target):
    effector = excavator.forward_kinematics2(joint_name="attachment_joint")

    print("")
    print("Virtual projection:")
    print("Target   :", format_euler_tuple(target))
    print("Effector :", format_euler_tuple(effector))

    excavator.inverse_kinematics(target[:3])

    proj_angle = np.sum(excavator.get_position_state_projected()[2:])
    print("IK: Projected Pitch:", format_angle_both(proj_angle))
    abs_error = target[4] - proj_angle
    print("IK: AbsPitch error:", format_angle_both(abs_error))
    rel_pitch_error0 = excavator.attachment + abs_error
    print("IK: RelPitch error:", format_angle_both(rel_pitch_error0))

    bc = excavator.attachment_joint.is_within_bounds(rel_pitch_error0)
    print("IK: Is Within bounds:", bc)
    if not bc:
        rel_pitch_error0 = excavator.attachment_joint.clip(rel_pitch_error0)
        print("IK: Clipped angle:", rel_pitch_error0)

    excavator.position_state[1][4] = rel_pitch_error0

    print()
    if SUPERVISOR:
        input("Press Enter to continue...")

    while True:
        excavator.frame = adapter.encoder["frame"]["angle"]
        excavator.boom = adapter.encoder["boom"]["angle"]
        excavator.arm = adapter.encoder["arm"]["angle"]
        excavator.attachment = adapter.encoder["attachment"]["angle"]

        print()
        print("Target:", format_euler_tuple(target))

        rel_error = excavator.get_position_error()[0]

        rel_frame_error = shortest_rotation(rel_error[1])
        rel_boom_error = rel_error[2]
        rel_arm_error = rel_error[3]
        rel_attachment_error = rel_error[4]

        power_setting_slew = motion_profile_slew.proportional_power(
            rel_frame_error)
        power_setting_boom = motion_profile_boom.proportional_power(
            rel_boom_error)
        power_setting_arm = motion_profile_arm.proportional_power_inverse(
            rel_arm_error)
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

        adapter.change(
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
            if SUPERVISOR:
                time.sleep(0.75)
            effector = excavator.forward_kinematics2(
                joint_name="attachment_joint")
            print("Target   :", format_euler_tuple(target))
            print("Effector :", format_euler_tuple(effector))
            if SUPERVISOR:
                input("Press Enter to continue...")
            break

        time.sleep(0.1)


try:
    adapter.enable_motion()

    # while True:
    for target in program:
        move_to_target(target)
except Exception as e:
    traceback.print_exception(type(e), e, e.__traceback__)
finally:
    adapter.disable_motion()
    adapter.stop()
    sys.exit(0)
