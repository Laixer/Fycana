import time
import logging

from pyglonax.motion import POWER_MAX, POWER_MIN
from pyglonax.excavator import Machine, Actuator

SLEEP_TIME = 1

logging.basicConfig(format='%(levelname)s %(message)s', level=logging.INFO)


class Test:
    def __init__(self):
        self.machine = Machine("./volvo_ec240cl.json")

    def stop(self):
        self.machine.disable_motion()

    def start(self):
        self.machine.start()
        self.machine.enable_motion()

        motion = self.machine.motion

        while True:
            logging.info("Testing actuator: boom up")
            motion.set([
                (Actuator.Boom, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: boom down")
            motion.set([
                (Actuator.Boom, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: arm up")
            motion.set([
                (Actuator.Arm, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: arm down")
            motion.set([
                (Actuator.Arm, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: attachment up")
            motion.set([
                (Actuator.Attachment, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: attachment down")
            motion.set([
                (Actuator.Attachment, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: slew up")
            motion.set([
                (Actuator.Slew, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: slew down")
            motion.set([
                (Actuator.Slew, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track left up")
            motion.set([
                (Actuator.TrackLeft, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track left down")
            motion.set([
                (Actuator.TrackLeft, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track right up")
            motion.set([
                (Actuator.TrackRight, POWER_MAX)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing actuator: track right down")
            motion.set([
                (Actuator.TrackRight, POWER_MIN)
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing drive: drive straight up")
            motion.straight_drive(POWER_MAX)

            time.sleep(SLEEP_TIME)

            logging.info("Testing drive: drive straight down")
            motion.straight_drive(POWER_MIN)

            time.sleep(SLEEP_TIME)

            logging.info("Testing all: actuators up")
            motion.change([
                (Actuator.Boom, POWER_MAX),
                (Actuator.Arm, POWER_MAX),
                (Actuator.Attachment, POWER_MAX),
                (Actuator.Slew, POWER_MAX),
                (Actuator.TrackLeft, POWER_MAX),
                (Actuator.TrackRight, POWER_MAX),
            ])

            time.sleep(SLEEP_TIME)

            logging.info("Testing all: actuators down")
            motion.change([
                (Actuator.Boom, POWER_MIN),
                (Actuator.Arm, POWER_MIN),
                (Actuator.Attachment, POWER_MIN),
                (Actuator.Slew, POWER_MIN),
                (Actuator.TrackLeft, POWER_MIN),
                (Actuator.TrackRight, POWER_MIN),
            ])

            time.sleep(SLEEP_TIME)


if __name__ == "__main__":
    prog = Test()

    try:
        prog.start()
    except KeyboardInterrupt:
        prog.stop()
