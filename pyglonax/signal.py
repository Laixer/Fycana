import datetime
import struct
from enum import IntEnum


class Signal:
    class SignalType(IntEnum):
        TEMPERATURE = 0x00
        ANGLE = 0x01
        SPEED = 0x02
        ALTITUDE = 0x03
        HEADING = 0x04
        RPM = 0x05
        ACCELERATION = 0x06
        PERCENT = 0x07
        COORDINATES = 0x08
        TIMESTAMP = 0x09
        POINT2D = 0x0A
        POINT3D = 0x0B
        COUNT = 0x0C

    def __init__(self, address, function, metric):
        self.address = address
        self.function = function
        self.metric = metric
        self.value = None

    def __str__(self):
        if self.metric == Signal.SignalType.TEMPERATURE:
            return f"0x{self.address:02X}:{self.function:02X} » Temperature: {self.value:.1f}°C"
        elif self.metric == Signal.SignalType.ANGLE:
            return (
                f"0x{self.address:02X}:{self.function:02X} » Angle: {self.value:.2f}rad"
            )
        elif self.metric == Signal.SignalType.SPEED:
            return (
                f"0x{self.address:02X}:{self.function:02X} » Speed: {self.value:.2f}m/s"
            )
        elif self.metric == Signal.SignalType.ALTITUDE:
            return f"0x{self.address:02X}:{self.function:02X} » Altitude: {self.value:.1f}m"
        elif self.metric == Signal.SignalType.HEADING:
            return (
                f"0x{self.address:02X}:{self.function:02X} » Heading: {self.value:.1f}°"
            )
        elif self.metric == Signal.SignalType.RPM:
            return f"0x{self.address:02X}:{self.function:02X} » RPM: {self.value}rpm"
        elif self.metric == Signal.SignalType.PERCENT:
            return f"0x{self.address:02X}:{self.function:02X} » Percent: {self.value}%"
        elif self.metric == Signal.SignalType.COORDINATES:
            return f"0x{self.address:02X}:{self.function:02X} » Coordinates: (Lat: {self.value[0]:.5f}, Long: {self.value[1]:.6f})"
        elif self.metric == Signal.SignalType.TIMESTAMP:
            return f"0x{self.address:02X}:{self.function:02X} » Timestamp: {self.value} UTC"
        elif self.metric == Signal.SignalType.COUNT:
            return f"0x{self.address:02X}:{self.function:02X} » Count: {self.value}"

    def from_bytes(data: bytes):
        address = struct.unpack_from(">i", data, offset=0)[0]
        function = struct.unpack_from(">i", data, offset=4)[0]

        metric = Signal.SignalType(data[8])

        signal = Signal(address, function, metric)

        if metric == Signal.SignalType.TEMPERATURE:
            signal.value = struct.unpack_from(">f", data, offset=9)[0]
        elif metric == Signal.SignalType.ANGLE:
            signal.value = struct.unpack_from(">f", data, offset=9)[0]
        elif metric == Signal.SignalType.SPEED:
            signal.value = struct.unpack_from(">f", data, offset=9)[0]
        elif metric == Signal.SignalType.ALTITUDE:
            signal.value = struct.unpack_from(">f", data, offset=9)[0]
        elif metric == Signal.SignalType.HEADING:
            signal.value = struct.unpack_from(">f", data, offset=9)[0]
        elif metric == Signal.SignalType.RPM:
            signal.value = struct.unpack_from(">i", data, offset=9)[0]
        elif metric == Signal.SignalType.ACCELERATION:
            value_x = struct.unpack_from(">f", data, offset=9)[0]
            value_y = struct.unpack_from(">f", data, offset=13)[0]
            value_z = struct.unpack_from(">f", data, offset=17)[0]
            signal.value = (value_x, value_y, value_z)
        elif metric == Signal.SignalType.PERCENT:
            signal.value = struct.unpack_from(">i", data, offset=9)[0]
        elif metric == Signal.SignalType.COORDINATES:
            latitude = struct.unpack_from(">f", data, offset=9)[0]
            longitude = struct.unpack_from(">f", data, offset=13)[0]
            signal.value = (latitude, longitude)
        elif metric == Signal.SignalType.TIMESTAMP:
            value = struct.unpack_from(">q", data, offset=9)[0]
            utc_dt = datetime.datetime.utcfromtimestamp(value)
            signal.value = utc_dt
        elif metric == Signal.SignalType.POINT2D:
            value_x = struct.unpack_from(">f", data, offset=9)[0]
            value_y = struct.unpack_from(">f", data, offset=13)[0]
            signal.value = (value_x, value_y)
        elif metric == Signal.SignalType.POINT3D:
            value_x = struct.unpack_from(">f", data, offset=9)[0]
            value_y = struct.unpack_from(">f", data, offset=13)[0]
            value_z = struct.unpack_from(">f", data, offset=17)[0]
            signal.value = (value_x, value_y, value_z)
        elif metric == Signal.SignalType.COUNT:
            signal.value = struct.unpack_from(">q", data, offset=9)[0]

        return signal
