import logging
import time

from pyglonax.robot import Robot
from pyglonax.adapter import Adapter

from . import motion


class ExcavatorActuator:
    Boom = 0
    Arm = 4
    Attachment = 5
    Slew = 1
    TrackLeft = 3
    TrackRight = 2


class ExcavatorAdapter(Adapter):
    encoder = {}
    engine = {}
    vms = {}
    gnss = {}
    signal_callback = None

    def signal_event(self, signal):
        # FUTURE: Use match statement
        if signal.address == 0x6B:
            self._on_boom_signal(signal)
        elif signal.address == 0x6C:
            self._on_arm_signal(signal)
        elif signal.address == 0x6A:
            self._on_frame_signal(signal)
        elif signal.address == 0x6D:
            self._on_attachment_signal(signal)
        elif signal.address == 0x0:
            self._on_engine_signal(signal)
        elif signal.address == 0x9E:
            self._on_host_metric(signal)
        # TODO: GNSS has address 0x01, which is not correct
        elif signal.address == 0x01:
            self._on_gnss_metric(signal)
        else:
            logging.debug("Unknown signal: %s", signal)

    def on_signal_update(self, func):
        self.signal_callback = func

    def _on_host_metric(self, signal):
        # TODO: Use hex addresses for functions
        if signal.function == 382:
            self.vms["memory"] = signal.value
            logging.debug(f"Host memory: {signal.value}%")
        elif signal.function == 383:
            self.vms["swap"] = signal.value
            logging.debug(f"Host swap: {signal.value}%")
        elif signal.function == 593:
            self.vms["cpu_1"] = signal.value
            logging.debug(f"Host CPU 1: {signal.value}%")
        elif signal.function == 594:
            self.vms["cpu_5"] = signal.value
            logging.debug(f"Host CPU 2: {signal.value}%")
        elif signal.function == 595:
            self.vms["cpu_15"] = signal.value
            logging.debug(f"Host CPU 3: {signal.value}%")
        elif signal.function == 0x1A4:
            self.vms["timestamp"] = signal.value
            logging.debug(f"Host timestamp: {signal.value}")
        elif signal.function == 0x1A5:
            self.vms["uptime"] = signal.value
            logging.debug(f"Host uptime: {signal.value}")
        else:
            logging.warn(f"Unknown host metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_gnss_metric(self, signal):
        if signal.function == 0:
            self.gnss["lat"] = signal.value[0]
            self.gnss["long"] = signal.value[1]
            logging.debug(
                f"GNSS Coordinates: (Lat: {signal.value[0]}, Long: {signal.value[1]})"
            )
        elif signal.function == 1:
            self.gnss["altitude"] = signal.value
            logging.debug(f"GNSS Altitude: {signal.value}")
        elif signal.function == 2:
            self.gnss["speed"] = signal.value
            logging.debug(f"GNSS Speed: {signal.value}")
        elif signal.function == 10:
            self.gnss["satellites"] = signal.value
            logging.debug(f"GNSS Satellites: {signal.value}")
        else:
            logging.warn(f"Unknown gnss metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_engine_signal(self, signal):
        if signal.function == 0:
            self.engine["rpm"] = signal.value
            logging.debug(f"Engine RPM: {signal.value}")
        elif signal.function == 1:
            self.engine["driver_demand"] = signal.value
            logging.debug(f"Engine Driver demand: {signal.value}%")
        elif signal.function == 2:
            self.engine["actual_engine"] = signal.value
            logging.debug(f"Engine Actual engine: {signal.value}%")
        else:
            logging.warn(f"Unknown engine metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_frame_signal(self, signal):
        if "frame" not in self.encoder:
            self.encoder["frame"] = {}
        if signal.function == 0:
            self.encoder["frame"]["position"] = signal.value
            self.encoder["frame"]["angle"] = signal.value
            logging.debug(f"Frame position: {signal.value:.3f}")
        else:
            logging.warn(f"Unknown frame metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_boom_signal(self, signal):
        if "boom" not in self.encoder:
            self.encoder["boom"] = {}
        if signal.function == 0:
            self.encoder["boom"]["position"] = signal.value
            self.encoder["boom"]["angle"] = signal.value - 1.047
            logging.debug(f"Boom position: {signal.value:.3f}")
        else:
            logging.warn(f"Unknown boom metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_arm_signal(self, signal):
        if "arm" not in self.encoder:
            self.encoder["arm"] = {}
        if signal.function == 0:
            self.encoder["arm"]["position"] = signal.value
            self.encoder["arm"]["angle"] = signal.value
            logging.debug(f"Arm position: {signal.value:.3f}")
        else:
            logging.warn(f"Unknown arm metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def _on_attachment_signal(self, signal):
        if "attachment" not in self.encoder:
            self.encoder["attachment"] = {}
        if signal.function == 0:
            self.encoder["attachment"]["position"] = signal.value
            self.encoder["attachment"]["angle"] = signal.value - 0.962
            logging.debug(f"Attachment position: {signal.value:.3f}")
        else:
            logging.warn(f"Unknown attachment metric: {signal}")
        if self.signal_callback:
            self.signal_callback(signal)

    def is_initialized(self):
        return (
            "attachment" in self.encoder
            and "arm" in self.encoder
            and "boom" in self.encoder
            and "frame" in self.encoder
        )

    def wait_until_initialized(self):
        while not self.is_initialized():
            time.sleep(0.1)

    def command(self, motion_list: list):
        default = [
            (ExcavatorActuator.Boom, motion.POWER_NEUTRAL),
            (ExcavatorActuator.Arm, motion.POWER_NEUTRAL),
            (ExcavatorActuator.Attachment, motion.POWER_NEUTRAL),
            (ExcavatorActuator.Slew, motion.POWER_NEUTRAL),
            (ExcavatorActuator.TrackLeft, motion.POWER_NEUTRAL),
            (ExcavatorActuator.TrackRight, motion.POWER_NEUTRAL),
        ]

        self.change(set(default) | set(motion_list))

    def straight_drive(self, power):
        self.change(
            [
                [ExcavatorActuator.TrackLeft, power],
                [ExcavatorActuator.TrackRight, power],
            ]
        )


class Excavator(Robot):
    # TODO: Remove this?
    @property
    def frame_joint(self):
        return self.get_joint_by_name("frame")

    # TODO: Remove this?
    @property
    def boom_joint(self):
        return self.get_joint_by_name("boom")

    # TODO: Remove this?
    @property
    def arm_joint(self):
        return self.get_joint_by_name("arm")

    # TODO: Remove this?
    @property
    def attachment_joint(self):
        return self.get_joint_by_name("attachment")

    @property
    def attachment(self):
        return self.get_position_state("attachment")

    @attachment.setter
    def attachment(self, angle):
        self.set_position_state("attachment", angle)
