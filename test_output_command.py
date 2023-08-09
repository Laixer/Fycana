#!/usr/bin/env python3

import time
import logging
import configparser

from pyglonax.motion import POWER_MAX, POWER_MIN
from pyglonax.excavator import ExcavatorAdapter, ExcavatorActuator


SLEEP_TIME = 1

config = configparser.ConfigParser()
config.read("config.ini")

host = config["glonax"]["host"]
port = config["glonax"]["port"]

logging.basicConfig(format="%(levelname)s %(message)s", level=logging.INFO)


class TestOutputCommand:
    """
    Test output command
    """

    def __init__(self, host, port):
        self.machine = ExcavatorAdapter(host, port)

    def stop(self):
        self.machine.disable_motion()
        self.machine.stop()

    def _reset(self):
        self.machine.change(
            [
                (ExcavatorActuator.Boom, 0),
                (ExcavatorActuator.Arm, 0),
                (ExcavatorActuator.Attachment, 0),
                (ExcavatorActuator.Slew, 0),
                (ExcavatorActuator.TrackLeft, 0),
                (ExcavatorActuator.TrackRight, 0),
            ]
        )

    def start(self):
        self.machine.start()
        self.machine.enable_motion()

        while True:
            logging.info("Testing actuator: boom up")
            self._reset()
            self.machine.change([(ExcavatorActuator.Boom, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: boom down")
            self._reset()
            self.machine.change([(ExcavatorActuator.Boom, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: arm up")
            self._reset()
            self.machine.change([(ExcavatorActuator.Arm, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: arm down")
            self._reset()
            self.machine.change([(ExcavatorActuator.Arm, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: attachment up")
            self._reset()
            self.machine.change([(ExcavatorActuator.Attachment, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: attachment down")
            self._reset()
            self.machine.change([(ExcavatorActuator.Attachment, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: slew up")
            self._reset()
            self.machine.change([(ExcavatorActuator.Slew, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: slew down")
            self._reset()
            self.machine.change([(ExcavatorActuator.Slew, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track left up")
            self._reset()
            self.machine.change([(ExcavatorActuator.TrackLeft, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track left down")
            self._reset()
            self.machine.change([(ExcavatorActuator.TrackLeft, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track right up")
            self._reset()
            self.machine.change([(ExcavatorActuator.TrackRight, POWER_MAX)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track right down")
            self._reset()
            self.machine.change([(ExcavatorActuator.TrackRight, POWER_MIN)])

            time.sleep(SLEEP_TIME)

            logging.info("Testing drive: drive straight up")
            self._reset()
            self.machine.change(
                [
                    (ExcavatorActuator.TrackLeft, POWER_MAX),
                    (ExcavatorActuator.TrackRight, POWER_MAX),
                ]
            )

            time.sleep(SLEEP_TIME)

            logging.info("Testing drive: drive straight down")
            self._reset()
            self.machine.change(
                [
                    (ExcavatorActuator.TrackLeft, POWER_MIN),
                    (ExcavatorActuator.TrackRight, POWER_MIN),
                ]
            )

            time.sleep(SLEEP_TIME)

            logging.info("Testing all: actuators up")
            self._reset()
            self.machine.change(
                [
                    (ExcavatorActuator.Boom, POWER_MAX),
                    (ExcavatorActuator.Arm, POWER_MAX),
                    (ExcavatorActuator.Attachment, POWER_MAX),
                    (ExcavatorActuator.Slew, POWER_MAX),
                    (ExcavatorActuator.TrackLeft, POWER_MAX),
                    (ExcavatorActuator.TrackRight, POWER_MAX),
                ]
            )

            time.sleep(SLEEP_TIME)

            logging.info("Testing all: actuators down")
            self._reset()
            self.machine.change(
                [
                    (ExcavatorActuator.Boom, POWER_MIN),
                    (ExcavatorActuator.Arm, POWER_MIN),
                    (ExcavatorActuator.Attachment, POWER_MIN),
                    (ExcavatorActuator.Slew, POWER_MIN),
                    (ExcavatorActuator.TrackLeft, POWER_MIN),
                    (ExcavatorActuator.TrackRight, POWER_MIN),
                ]
            )

            time.sleep(SLEEP_TIME)


if __name__ == "__main__":
    program = TestOutputCommand(host, port)
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
