import logging
import time
import grpc
import threading

from enum import Enum


from . import vms_pb2, vms_pb2_grpc


class Adapter:
    """
    Adapter to remote machine
    """

    class ConnectionState(Enum):
        """
        Connection state
        """

        DISCONNECTED = 0
        CONNECTED = 1

    last_signal = 0
    status = ConnectionState.DISCONNECTED

    def signal_event(self, signal):
        """
        Signal event handler.

        Args:
            signal (Signal): signal object.
        """
        raise NotImplementedError()

    def _on_signal(self):
        logging.debug("Listening for incoming signals")

        try:
            for signal in self._stub.ListenSignal(vms_pb2.Empty()):
                if self._event.is_set():
                    break

                self.last_signal = time.time()
                self.status = self.ConnectionState.CONNECTED
                self.signal_event(signal)
        except grpc.RpcError as e:
            match e.code():
                case grpc.StatusCode.UNAVAILABLE:
                    logging.error("Unable to connect to remote machine: %s", self._host)
                case grpc.StatusCode.CANCELLED:
                    logging.debug("Signal listener cancelled")
                case _:
                    logging.error("Error: %s", e.code())

        self.status = self.ConnectionState.DISCONNECTED
        logging.debug("Signal listener stopped")

    def __init__(self, host="localhost:50051"):
        """
        Initializes the excavator interface.

        Args:
            host (str): remote machine host.
        """
        self._host = host
        self._setup()

    def _setup(self):
        self._channel = grpc.insecure_channel(self._host)
        self._stub = vms_pb2_grpc.VehicleManagementStub(self._channel)

        self._event = threading.Event()
        self._signal_thread = threading.Thread(target=self._on_signal, args=())

    @property
    def host(self):
        return self._host

    @property
    def signal_elapsed_time(self) -> float:
        """Returns the elapsed time in seconds since the last signal"""
        return time.time() - self.last_signal

    def _commit(self, motion_request):
        try:
            return self._stub.MotionCommand(motion_request, timeout=1)
        except grpc.RpcError as e:
            match e.code():
                case grpc.StatusCode.UNAVAILABLE:
                    self.status = self.ConnectionState.DISCONNECTED
                    logging.error("Unable to connect to remote machine: %s", self._host)
                case grpc.StatusCode.CANCELLED:
                    logging.debug("Signal motion command")
                case _:
                    logging.error("Error: %s", e.code())

    def change(self, motion_list):
        changes = map(
            lambda a: vms_pb2.Motion.ChangeSet(actuator=a[0], value=a[1]), motion_list
        )
        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.CHANGE, changes=changes))

    def disable_motion(self):
        """Disables motion"""
        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.STOP_ALL))

    def enable_motion(self):
        """Enables motion"""
        self._commit(vms_pb2.Motion(type=vms_pb2.Motion.RESUME_ALL))

    def restart(self):
        """Restarts the machine"""
        logging.debug("Restarting machine")
        self.stop()
        self._setup()
        self.start()

    def start(self):
        """Starts the machine"""
        logging.debug("Starting machine")
        self._signal_thread.start()

    def stop(self):
        """Stops the machine"""
        logging.debug("Stopping machine")
        self._event.set()
        self._channel.close()
        self._signal_thread.join()

    def idle(self):
        while self.status == self.ConnectionState.CONNECTED:
            time.sleep(0.1)
