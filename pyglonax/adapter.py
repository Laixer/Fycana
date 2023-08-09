import logging
import time
import socket
import struct
import threading
from enum import Enum

from .signal import Signal
from .motion import Motion


# TODO: Move to transport.py or protocol.py or something
def build_protocol(message, bytes):
    PROTOCOL_HEADER = [76, 88, 82]
    PROTOCOL_VERSION = 0x1

    buffer = bytearray(PROTOCOL_HEADER)
    buffer.append(PROTOCOL_VERSION)
    buffer.append(message)

    length = len(bytes)
    buffer += struct.pack(">H", length)
    buffer += bytes

    return buffer


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

        HEADER_SIZE = 3 + 1 + 1 + 2

        while not self._event.is_set():
            data = self.socket.recv(HEADER_SIZE)
            if not data:
                break

            if len(data) != HEADER_SIZE:
                logging.debug("Invalid header size")
                continue

            if data[0] != 76 or data[1] != 88 or data[2] != 82:
                logging.debug("Invalid header")
                continue

            version = data[3]
            if version != 0x1:
                logging.debug("Invalid version")
                continue

            message = data[4]
            if message != 0x31:  # 0x31 = Signal
                logging.debug("Invalid message")
                continue

            payload_length = int.from_bytes(data[5:7], byteorder="big", signed=False)
            if payload_length > 4096:
                logging.debug("Invalid payload length")
                continue

            payload = self.socket.recv(payload_length)
            if not payload:
                break

            signal = Signal.from_bytes(payload)

            self.last_signal = time.time()
            self.status = self.ConnectionState.CONNECTED
            self.signal_event(signal)

        self.status = self.ConnectionState.DISCONNECTED
        logging.debug("Signal listener stopped")

    def __init__(self, host="localhost", port=30051):
        """
        Initializes the excavator interface.

        Args:
            host (str): remote machine host.
        """
        self._host = host
        if isinstance(port, str):
            self._port = int(port)
        else:
            self._port = port
        self._setup()

    def _setup(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self._host, self._port))

        self._helo()

        self._event = threading.Event()
        self._signal_thread = threading.Thread(target=self._on_signal, args=())

    @property
    def host(self):
        return self._host

    @property
    def signal_elapsed_time(self) -> float:
        """Returns the elapsed time in seconds since the last signal"""
        return time.time() - self.last_signal

    def _helo(self):
        proto_bytes = build_protocol(0x10, bytearray(b"Woei"))
        self.socket.sendall(proto_bytes)

    def _shutdown(self):
        proto_bytes = build_protocol(0x11, bytearray())
        self.socket.sendall(proto_bytes)
        self.socket.shutdown(socket.SHUT_RDWR)

    def change(self, motion_list):
        """Changes motion"""
        motion = Motion(Motion.MotionType.CHANGE, motion_list)
        proto_bytes = build_protocol(Motion.PROTOCOL_MESSAGE, motion.to_bytes())
        self.socket.sendall(proto_bytes)

    def disable_motion(self):
        """Disables motion"""
        motion = Motion(Motion.MotionType.STOP_ALL)
        proto_bytes = build_protocol(Motion.PROTOCOL_MESSAGE, motion.to_bytes())
        self.socket.sendall(proto_bytes)

    def enable_motion(self):
        """Enables motion"""
        motion = Motion(Motion.MotionType.RESUME_ALL)
        proto_bytes = build_protocol(Motion.PROTOCOL_MESSAGE, motion.to_bytes())
        self.socket.sendall(proto_bytes)

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
        self._shutdown()
        self._event.set()
        self._signal_thread.join()
        self.socket.close()

    def is_terminated(self):
        """Returns True if the machine is running"""
        return self._event.is_set()

    def idle(self):
        while not self.is_terminated():
            time.sleep(0.1)
