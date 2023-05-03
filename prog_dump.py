import logging

from pyglonax.excavator import ExcavatorAdapter
from pyglonax.util import get_config

config = get_config()

logging.basicConfig(format="%(levelname)s %(message)s", level=logging.DEBUG)


class Dump:
    """
    Diagnose the machine
    """

    def __init__(self, host):
        self.machine = ExcavatorAdapter(host=host)

    def stop(self):
        self.machine.stop()

    def start(self):
        self.machine.start()
        self.machine.idle()


if __name__ == "__main__":
    program = Dump(host=config["GLONAX_HOST"])
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
