import os
import sys
import time
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import get_config

config = get_config()

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})


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

tolerance = 0.005

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

# adapter.stop()
# sys.exit(0)

### Kinematics test

program = np.array([[7.73, 0.00, 2.29], [5.35, 0.00, 1.93], [0.00, 7.73, 2.29]])

print("Program:", program)

# adapter.stop()
# sys.exit(0)

print("Starting program")

for target in program:
    effector = excavator.forward_kinematics(joint_name="attachment_joint")
    print("")
    print("End effector:", effector)
    print("Target:", target)

    excavator.inverse_kinematics(target)

    while True:
        excavator.frame = adapter.encoder["frame"]["angle"]
        excavator.boom = adapter.encoder["boom"]["angle"]
        excavator.arm = adapter.encoder["arm"]["angle"]
        excavator.attachment = adapter.encoder["attachment"]["angle"]

        print()
        error = excavator.get_position_error()[0]
        print("Relative error:", error[1:4])

        power_setting_slew = motion_profile_slew.proportional_power(error[1])
        print("Power setting Slew:", int(power_setting_slew))
        power_setting_boom = motion_profile_boom.proportional_power(error[2])
        print("Power setting Boom:", int(power_setting_boom))
        power_setting_arm = motion_profile_arm.proportional_power(error[3])
        print("Power setting Arm:", int(power_setting_arm))

        adapter.change(
            [
                (ExcavatorActuator.Slew, int(power_setting_slew)),
                (ExcavatorActuator.Boom, int(power_setting_boom)),
                (ExcavatorActuator.Arm, int(power_setting_arm)),
            ]
        )

        if excavator.is_objective_reached(tolerance):
            print("Objective reached")
            input("Press Enter to continue...")
            break
        else:
            print("Objective not reached")

        time.sleep(0.1)

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

adapter.stop()
sys.exit(0)
