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


# TODO: Remove this
class Motion:
    def __init__(self, channel):
        self._stub = vms_pb2_grpc.VehicleManagementStub(channel)

    def _commit(self, motion_request):
        try:
            return self._stub.MotionCommand(motion_request, timeout=1)
        except grpc.RpcError as e:
            logging.error("Error sending motion command: %s", e.details())

    def change(self, motion_list):
        changes = map(
            lambda a: vms_pb2.Motion.ChangeSet(actuator=a[0], value=a[1]), motion_list
        )

        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.CHANGE, changes=changes))

    def stop_all(self):
        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.STOP_ALL))

    def resume_all(self):
        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.RESUME_ALL))


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
