import time
import math
import ec240cl
import logging
import ik

from pyglonax.excavator import Machine
from pyglonax.machine import MotionProfile

logging.basicConfig(format='%(levelname)s %(message)s', level=logging.DEBUG)


class TestProgram:
    def __init__(self):
        # self.machine = Machine(definition="./volvo_ec240cl.json", host="192.168.240.100")
        self.machine = Machine(definition="./volvo_ec240cl.json")

    def stop(self):
        self.machine.disable_motion()
        self.machine.stop()

    def start(self):
        self.machine.start()
        self.machine.enable_motion()

        motion_profile_slew = MotionProfile(10_000, 12_000, 0.02, False)
        motion_profile_boom = MotionProfile(15_000, 12_000, 0.02, True)
        motion_profile_arm = MotionProfile(15_000, 12_000, 0.02, False)

        solver = ik.InverseKinematics(self.machine.boom['length'], self.machine.arm['length'])
        res = solver.solve([5.21, 0, 0])

        while True:
            # boom_point = self.machine.boom_point()
            # if boom_point is not None:
            #     print("Boom point:\t\t {:>.2f}, {:>.2f}".format(
            #         boom_point[0], boom_point[1]))

            # effector_point_flat = self.machine.effector_point_flat()
            # if effector_point_flat is not None:
            #     print("Effector point 2D:\t {:>.2f}, {:>.2f}".format(
            #         effector_point_flat[0], effector_point_flat[1]))

            effector_point = self.machine.effector_point()
            if effector_point is not None:
                print("Effector point 3D:\t {:>.2f}, {:>.2f}, {:>.2f}".format(
                    effector_point[0], effector_point[1], effector_point[2]))

                error_angle_slew = res[0] - self.machine.encoder['frame']['angle_abs']
                error_angle_boom = res[1] - self.machine.encoder['boom']['angle_abs']
                error_angle_arm = res[2] - self.machine.encoder['arm']['angle_abs']

                print("Errors Frame: {:>.2f}\tBoom: {:>.2f}\tArm: {:>.2f}".format(
                    error_angle_slew, error_angle_boom, error_angle_arm))

                # if abs(error_angle_slew) < 0.02 and abs(error_angle_boom) < 0.02 and abs(error_angle_arm) < 0.02:
                # break

                # let mut motion_vector = vec![];

                power_boom = motion_profile_boom.proportional_power_inverse(error_angle_boom)
                power_arm = motion_profile_arm.proportional_power(error_angle_arm)
                power_slew = motion_profile_slew.proportional_power(error_angle_slew)

                print("Power Frame: {}\tBoom: {}\tArm: {}".format(
                    power_slew, power_boom, power_arm))

                # mo_cmd = [
                #     (Actuator.Boom, power_boom),
                #     (Actuator.Slew, power_slew),
                #     (Actuator.Arm, power_arm)
                # ]

                # motion.set([
                #     (Actuator.Boom, machine.POWER_MAX)
                # ])

                # let power = super:: consts: : MOTION_PROFILE_BOOM.proportional_power_inverse(error_angle_boom)
                # motion_vector.push((super:: Actuator: : Boom, power))

                # let power = super:: consts: : MOTION_PROFILE_ARM.proportional_power(error_angle_arm)
                # motion_vector.push((super:: Actuator: : Arm, power))

                # let power = super:: consts: : MOTION_PROFILE_SLEW.proportional_power(error_angle_slew)
                # motion_vector.push((super:: Actuator: : Slew, power))

            time.sleep(0.1)


# motion_profile_boom = machine.MotionProfile(15_000, 12_000, 0.02, True)
# print(motion_profile_boom.proportional_power_inverse(0.17))


if __name__ == "__main__":
    program = TestProgram()
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
