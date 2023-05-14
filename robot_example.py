import os
import sys
import time
import traceback
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import get_config

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

# effector = excavator.forward_kinematics(joint_name="attachment_joint")
# print("End effector:", effector)

# adapter.disable_motion()
# adapter.stop()
# sys.exit(0)

### Kinematics test

program = np.array(
    [
        [5.56, 0.00, 1.65, 2.3911],
        # begin loop
        [6.27, 0.00, 3.58, 0.2792],
        [7.63, 0.00, 4.45, 0.4363],
        [8.05, 0.00, 2.19, 0.7330],
        [7.14, 0.00, 1.44, 2.2340],
        [5.85, 0.00, 1.85, 3.1415],
        [3.55, 4.60, 2.58, 3.0019],
        [4.85, 6.26, 1.96, -0.192],
        # begin loop
        [6.27, 0.00, 3.58, 0.2792],
        # [5.22, 6.60, 2.05],
        # [5.95, 0.00, 1.29],
    ]
)

print("Program:", program)

# adapter.disable_motion()
# adapter.stop()
# sys.exit(0)

print("Starting program")
input("Press Enter to continue...")


def move_to_target(target):
    effector = excavator.forward_kinematics(joint_name="attachment_joint")

    print("")
    print("End effector:", effector)
    print("Target:", target)

    excavator.inverse_kinematics(target[:3])

    curr_angle = np.sum(excavator.position_state[0][2:])
    print("Current Pitch: {:.2f}".format(np.rad2deg(curr_angle)))

    proj_angle = np.sum(excavator.position_state[1][2:])
    print("Projected Pitch: {:.2f}".format(np.rad2deg(proj_angle)))

    print("Expected Pitch: {:.2f}".format(np.rad2deg(target[3])))

    error = target[3] - proj_angle
    print("Pitch Error: {:.2f}".format(np.rad2deg(error)))

    rel_angle = excavator.attachment + error
    print("Relative Pitch: {:.2f}".format(np.rad2deg(rel_angle)))

    bc = excavator.attachment_joint.is_within_bounds(rel_angle)
    print("Within bounds:", bc)
    if not bc:
        rel_angle = excavator.attachment_joint.clip(rel_angle)
        print("Clipped angle:", rel_angle)

    # input("Press Enter to continue...")

    while True:
        excavator.frame = adapter.encoder["frame"]["angle"]
        excavator.boom = adapter.encoder["boom"]["angle"]
        excavator.arm = adapter.encoder["arm"]["angle"]
        excavator.attachment = adapter.encoder["attachment"]["angle"]

        print()
        error = excavator.get_position_error()[0]
        print("Relative error:", error[1:4])

        rel_error = rel_angle - excavator.attachment
        print("Relative Pitch error: {:.3f}".format(rel_error))

        power_setting_slew = motion_profile_slew.proportional_power(error[1])
        print("Power setting Slew:", int(power_setting_slew))
        power_setting_boom = motion_profile_boom.proportional_power(error[2])
        print("Power setting Boom:", int(power_setting_boom))
        power_setting_arm = motion_profile_arm.proportional_power_inverse(error[3])
        print("Power setting Arm:", int(power_setting_arm))
        power_setting = motion_profile_attachment.proportional_power(rel_error)
        print("Power setting Attachment:", int(power_setting))

        adapter.change(
            [
                (ExcavatorActuator.Slew, int(power_setting_slew)),
                (ExcavatorActuator.Boom, int(power_setting_boom)),
                (ExcavatorActuator.Arm, int(power_setting_arm)),
                (ExcavatorActuator.Attachment, int(power_setting)),
            ]
        )

        if excavator.is_objective_reached(tolerance) and abs(rel_error) < tolerance:
            print("Objective reached")
            # input("Press Enter to continue...")
            break
        else:
            print("Objective not reached")

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

### Attachment test

want = np.deg2rad(110)

curr_angle = (np.sum(excavator.position_state[0][2:]),)
print("Angle:", curr_angle[0])

error = want - curr_angle[0]
print("Error:", error)

rel_angle = excavator.attachment + error
print("Relative angle:", rel_angle)

bc = excavator.attachment_joint.is_within_bounds(rel_angle)
print("Within bounds:", bc)
if not bc:
    rel_angle = excavator.attachment_joint.clip(rel_angle)
    print("Clipped angle:", rel_angle)

input("Press Enter to continue...")

adapter.enable_motion()

while True:
    excavator.frame = adapter.encoder["frame"]["angle"]
    excavator.boom = adapter.encoder["boom"]["angle"]
    excavator.arm = adapter.encoder["arm"]["angle"]
    excavator.attachment = adapter.encoder["attachment"]["angle"]

    print()
    rel_error = rel_angle - excavator.attachment
    print("Relative error:", rel_error)

    power_setting = motion_profile_attachment.proportional_power(rel_error)
    print("Power setting:", int(power_setting))

    adapter.change(
        [
            (ExcavatorActuator.Attachment, int(power_setting)),
        ]
    )

    if abs(rel_error) < 0.02:
        print("Objective reached")
        break

    time.sleep(0.1)

adapter.disable_motion()
adapter.stop()
sys.exit(0)
