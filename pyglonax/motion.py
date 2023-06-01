import grpc
import logging

from . import vms_pb2
from . import vms_pb2_grpc

""" Maximum power setting. """
POWER_MAX = 32_000
""" Neutral power setting """
POWER_NEUTRAL = 0
""" Minimum power setting """
POWER_MIN = -32_000


class MotionProfile:
    def __init__(self, scale, offset, lower_bound, inverse):
        self.scale = scale
        self.offset = offset
        self.lower_bound = lower_bound
        self.inverse = inverse

    def power(self, value) -> int:
        if self.inverse:
            return int(self.proportional_power_inverse(value))
        return int(self.proportional_power(value))

    def proportional_power(self, value) -> int:
        if abs(value) > self.lower_bound:
            power = self.offset + min((abs(value) * self.scale), 32_767 - self.offset)
            if value < 0:
                return -power
            else:
                return power
        else:
            return 0

    def proportional_power_inverse(self, value) -> int:
        if abs(value) > self.lower_bound:
            power = value * self.scale

            if value > 0:
                return max(-power, -(32_767 - self.offset)) - self.offset
            else:
                return min(-power, 32_767 - self.offset) + self.offset
        else:
            return 0
