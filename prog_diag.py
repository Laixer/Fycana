import logging

from pyglonax.excavator import ExcavatorAdapter

logging.basicConfig(format='%(levelname)s %(message)s', level=logging.DEBUG)


class Diagnose:
    """
    Diagnose the machine
    """

    def __init__(self, host="localhost:50051"):
        self.machine = ExcavatorAdapter(host=host)

    def stop(self):
        self.machine.stop()

    def start(self):
        self.machine.start()
        self.machine.idle()


if __name__ == "__main__":
    # program = Diagnose("192.168.240.100:50051")
    program = Diagnose()
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
