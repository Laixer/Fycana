from enum import IntEnum

""" Maximum power setting. """
POWER_MAX = 32_000
""" Neutral power setting """
POWER_NEUTRAL = 0
""" Minimum power setting """
POWER_MIN = -32_000


class Motion:
    PROTOCOL_MESSAGE = 0x20

    class MotionType(IntEnum):
        STOP_ALL = 0x00
        RESUME_ALL = 0x01
        STRAIGHT_DRIVE = 0x05
        CHANGE = 0x10

    def __init__(self, type, changes=[]):
        self.type = type
        self._changes = changes

    def to_bytes(self):
        buffer = bytearray()

        buffer.append(self.type)
        buffer.append(len(self._changes))

        for actuator, value in self._changes:
            buffer += bytearray(actuator.to_bytes(2, byteorder="big", signed=False))
            buffer += bytearray(value.to_bytes(2, byteorder="big", signed=True))

        return buffer


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
