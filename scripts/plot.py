import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use('TkAgg')

# plt.plot([1, 2, 3, 4], [1, 4, 9, 16], 'ro')
# plt.plot([4, 9, 1, 7], 'ro')
# plt.ylabel('some numbers')
# plt.show()


def logistic(f, x, bounds=(0.02, 32_767.0)):
    if abs(x) > bounds[0]:
        return (1 / (1 + np.exp(-f*x)))*(2*bounds[1])-(bounds[1])
    else:
        return 0


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


motion_profile_slew = MotionProfile(10_000, 12_000, 0.02, False)
motion_profile_boom = MotionProfile(15_000, 12_000, 0.02, True)
motion_profile_arm = MotionProfile(15_000, 12_000, 0.02, False)


plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [motion_profile_slew.proportional_power(x) for x in np.arange(-np.pi, np.pi, 0.01)], color='blue')
# plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [motion_profile_boom.proportional_power_inverse(x) for x in np.arange(-np.pi, np.pi, 0.01)], color='purple')
plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [motion_profile_arm.proportional_power(x) for x in np.arange(-np.pi, np.pi, 0.01)], color='red')
# plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [logistic(1, x) for x in np.arange(-np.pi, np.pi, 0.01)], color='green')
plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [logistic(5, x) for x in np.arange(-np.pi, np.pi, 0.01)], color='brown')
# plt.plot([x for x in np.arange(-np.pi, np.pi, 0.01)], [logistic(-5, x) for x in np.arange(-np.pi, np.pi, 0.01)], color='green')

plt.show()
