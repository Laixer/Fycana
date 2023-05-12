import math


class InverseKinematics:
    def __init__(self, l1, l2):
        self.l1 = l1
        self.l2 = l2

    def solve(self, target):
        l4 = math.sqrt((target[0] ** 2) + (target[2] ** 2))
        l5 = math.sqrt((l4 ** 2) + (target[1] ** 2))

        theta_0 = math.atan2(target[2], target[0])

        theta_1 = math.atan2(target[1], l4) + math.acos(((self.l1 ** 2) +
                                                        (l5 ** 2) - (self.l2 ** 2)) / (2.0 * self.l1 * l5))

        theta_2 = math.acos(
            ((self.l1 ** 2) + (self.l2 ** 2) - (l5 ** 2)) / (2.0 * self.l1 * self.l2))

        theta_2 = math.pi - theta_2

        if l5 >= self.l1 + self.l2:
            raise ValueError("Out of reach")
        else:
            return (theta_0, theta_1, -theta_2)
