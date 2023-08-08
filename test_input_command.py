#!/usr/bin/env python3

import logging
import configparser

from pyglonax.excavator import ExcavatorAdapter


config = configparser.ConfigParser()
config.read("config.ini")

host = config["glonax"]["host"]
port = config["glonax"]["port"]

logging.basicConfig(format="%(levelname)s %(message)s", level=logging.DEBUG)


class TestInputCommand:
    """
    Diagnose the machine
    """

    def __init__(self, host, port):
        self.machine = ExcavatorAdapter(host, port)

    def stop(self):
        self.machine.stop()

    def start(self):
        self.machine.start()
        self.machine.idle()


if __name__ == "__main__":
    program = TestInputCommand(host, port)
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
