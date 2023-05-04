import logging
import time

from pyglonax.robot import Robot
from pyglonax.adapter import Adapter

from . import motion


class ExcavatorActuator:
    """
    Acutators of the excavator machine
    """

    Boom = 0
    Arm = 4
    Attachment = 5
    Slew = 1
    TrackLeft = 3
    TrackRight = 2


class ExcavatorAdapter(Adapter):
    """
    Adapter to remote excavator
    """

    encoder = {}
    engine = {}

    def signal_event(self, signal):
        """
        Signal event handler.

        Args:
            signal (Signal): signal object.
        """

        match signal.address:
            case 0x6B:
                self._on_boom_signal(signal)
            case 0x6C:
                self._on_arm_signal(signal)
            case 0x6A:
                self._on_frame_signal(signal)
            case 0x6D:
                self._on_attachment_signal(signal)
            case 0x0:
                self._on_engine_signal(signal)
            case _:
                logging.debug("Unknown signal: %s", signal)

    @property
    def host(self):
        return self._host

    def _on_engine_signal(self, signal):
        if signal.function == 0:
            self.engine["rpm"] = signal.rpm
            logging.debug(f"RPM: {signal.rpm}")
        if signal.function == 1:
            self.engine["driver_demand"] = signal.percent
            logging.debug(f"Driver demand: {signal.percent}%")
        if signal.function == 2:
            self.engine["actual_engine"] = signal.percent
            logging.debug(f"Actual engine: {signal.percent}%")

    def _on_frame_signal(self, signal):
        if "frame" not in self.encoder:
            self.encoder["frame"] = {}
        if signal.function == 0:
            self.encoder["frame"]["position"] = signal.angle
            self.encoder["frame"]["angle"] = signal.angle
            logging.debug(f"Frame position: {signal.angle:.21}")

    def _on_boom_signal(self, signal):
        if "boom" not in self.encoder:
            self.encoder["boom"] = {}
        if signal.function == 0:
            self.encoder["boom"]["position"] = signal.angle
            self.encoder["boom"]["angle"] = signal.angle - 1.047
            logging.debug(f"Boom position: {signal.angle:.1f}")

    def _on_arm_signal(self, signal):
        if "arm" not in self.encoder:
            self.encoder["arm"] = {}
        if signal.function == 0:
            self.encoder["arm"]["position"] = signal.angle
            self.encoder["arm"]["angle"] = signal.angle
            logging.debug(f"Arm position: {signal.angle:.1f}")

    def _on_attachment_signal(self, signal):
        if "attachment" not in self.encoder:
            self.encoder["attachment"] = {}
        if signal.function == 0:
            self.encoder["attachment"]["position"] = signal.angle
            self.encoder["attachment"]["angle"] = signal.angle
            logging.debug(f"Attachment position: {signal.angle:.1f}")

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
    """
    A robot excavator
    """

    # TODO: Remove this?
    @property
    def frame_joint(self):
        return self.get_joint_by_name("frame_joint")

    # TODO: Remove this?
    @property
    def boom_joint(self):
        return self.get_joint_by_name("boom_joint")

    # TODO: Remove this?
    @property
    def arm_joint(self):
        return self.get_joint_by_name("arm_joint")

    # TODO: Remove this?
    @property
    def attachment_joint(self):
        return self.get_joint_by_name("attachment_joint")

    @property
    def frame(self):
        return self.get_position_state("frame_joint")

    @frame.setter
    def frame(self, angle):
        self.set_position_state("frame_joint", angle)

    @property
    def boom(self):
        return self.get_position_state("boom_joint")

    @boom.setter
    def boom(self, angle):
        self.set_position_state("boom_joint", angle)

    @property
    def arm(self):
        return self.get_position_state("arm_joint")

    @arm.setter
    def arm(self, angle):
        self.set_position_state("arm_joint", angle)

    @property
    def attachment(self):
        return self.get_position_state("attachment_joint")

    @attachment.setter
    def attachment(self, angle):
        self.set_position_state("attachment_joint", angle)
