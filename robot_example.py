import os
import sys
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


def format_angle(value):
    return "{:.2f}°".format(np.rad2deg(value))


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

# program = np.load(file="model/default_trainnig_v1.npy")
excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])

print(excavator)

# sys.exit(0)

adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])

# excavator.plot_robot()

# for x in adapter.body.edges(data=True):
# print(x)

adapter.start()
adapter.wait_until_initialized()

print("Machine initialized")

excavator.frame = adapter.encoder["frame"]["angle"]
excavator.boom = adapter.encoder["boom"]["angle"]
excavator.arm = adapter.encoder["arm"]["angle"]
excavator.attachment = adapter.encoder["attachment"]["angle"]

# effector = excavator.forward_kinematics2(joint_name="attachment_joint")
# print("End effector:", format_euler_tuple(effector))

# adapter.disable_motion()
# adapter.stop()
# sys.exit(0)

### Kinematics test


program = np.array(
    [
        [5.56, 0.00, 1.65, 0.00, 2.3911, 0.00],
        # begin loop
        [6.27, 0.00, 3.58, 0.00, 0.2792, 0.00],
        [7.63, 0.00, 4.45, 0.00, 0.4363, 0.00],
        [8.05, 0.00, 2.19, 0.00, 0.7330, 0.00],
        [7.14, 0.00, 1.44, 0.00, 2.2340, 0.00],
        [5.85, 0.00, 1.85, 0.00, 3.1415, 0.00],
        [3.55, 4.60, 2.58, 0.00, 3.0019, 0.00],
        [4.85, 6.26, 1.96, 0.00, -0.192, 0.00],
        # begin loop
        [6.27, 0.00, 3.58, 0.00, 0.2792, 0.00],
    ]
)

# program = np.array(
#     [
#         [5.56, 0.00, 1.65, 0.00, 2.39, 0.00],
#         # begin loop
#         [6.27, 0.00, 3.58, 0.00, 0.28, 0.00],
#         [7.63, 0.00, 4.45, 0.00, 0.45, 0.00],
#         [8.05, 0.00, 2.19, 0.00, 0.73, 0.00],
#         [7.14, 0.00, 1.44, 0.00, 2.23, 0.00],
#         [5.85, 0.00, 1.85, 0.00, 3.14, 0.00],
#         [3.55, 0.00, 2.58, 0.00, 3.00, 0.00],
#         [4.85, 6.26, 1.96, 0.00, -0.19, 0.00],
#         [5.79, 0.43, 1.55, 0.00, 2.14, 0.00],
#         [7.68, 0.57, 4.38, 0.00, 1.03, 0.00],
#         [8.53, 0.70, 1.81, 0.00, 0.59, 0.00],
#         [7.06, 0.57, 1.34, 0.00, 2.47, 0.00],
#         [3.93, 0.33, 2.45, 0.00, 3.10, 0.00],
#         [2.44, 3.09, 2.45, 0.00, 3.10, 0.00],
#         [4.78, 6.05, 3.53, 0.00, 2.67, 0.00],
#         [5.32, 6.71, 2.24, 0.00, 0.68, 0.00],
#         [3.63, 4.56, 2.37, 0.00, 2.97, 0.00],
#         [4.69, 5.49, 1.62, 0.00, 0.63, 0.00],
#         [0.10, 4.89, 2.63, 0.00, 2.12, 0.00],
#         [0.10, 5.58, 1.35, 0.00, 2.16, 0.00],
#         [5.39, 0.31, 3.40, 0.00, 1.80, 0.00],
#         [7.95, 0.47, 2.22, 0.00, 0.86, 0.00],
#         [3.95, 0.24, 2.05, 0.00, 3.03, 0.00],
#         [0.65, 3.90, 2.05, 0.00, 3.03, 0.00],
#         [3.84, 4.69, 2.18, 0.00, 2.60, 0.00],
#         # end loop
#         [5.56, 0.00, 1.65, 0.00, 2.39, 0.00],
#     ]
# )

print("Program:")
for idx, target in enumerate(program):
    print(f"{idx}", format_euler_tuple(target))

# adapter.disable_motion()
# adapter.stop()
# sys.exit(0)

print()
print("Starting program")
input("Press Enter to continue...")


def move_to_target(target):
    effector = excavator.forward_kinematics2(joint_name="attachment_joint")

    print("")
    print("Target   :", format_euler_tuple(target))
    print("Effector :", format_euler_tuple(effector))

    excavator.inverse_kinematics(target[:3])

    proj_angle = np.sum(excavator.get_position_state_projected()[2:])
    print("IK: Projected Pitch:", format_angle(proj_angle))
    abs_error = target[4] - proj_angle
    print("IK: AbsPitch error:", format_angle(abs_error))
    rel_pitch_error0 = excavator.attachment + abs_error
    print("IK: RelPitch error", format_angle(rel_pitch_error0))

    bc = excavator.attachment_joint.is_within_bounds(rel_pitch_error0)
    print("IK: Is Within bounds:", bc)
    if not bc:
        rel_pitch_error0 = excavator.attachment_joint.clip(rel_pitch_error0)
        print("IK: Clipped angle:", rel_pitch_error0)

    excavator.position_state[1][4] = rel_pitch_error0

    print()
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

        power_setting_slew = motion_profile_slew.proportional_power(rel_frame_error)
        power_setting_boom = motion_profile_boom.proportional_power(rel_boom_error)
        power_setting_arm = motion_profile_arm.proportional_power_inverse(rel_arm_error)
        power_setting = motion_profile_attachment.proportional_power(
            rel_attachment_error
        )

        print(
            "{:<15}".format("Frame"),
            "  Error: {:>7}".format(format_angle(rel_frame_error)),
            "  Power: {:>6d}".format(int(power_setting_slew)),
        )
        print(
            "{:<15}".format("Boom"),
            "  Error: {:>7}".format(format_angle(rel_boom_error)),
            "  Power: {:>6d}".format(int(power_setting_boom)),
        )
        print(
            "{:<15}".format("Arm"),
            "  Error: {:>7}".format(format_angle(rel_arm_error)),
            "  Power: {:>6d}".format(int(power_setting_arm)),
        )
        print(
            "{:<15}".format("Attachment"),
            "  Error: {:>7}".format(format_angle(rel_attachment_error)),
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
