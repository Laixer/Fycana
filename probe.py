#!/usr/bin/env python3

import time
import logging
import requests
import configparser

from pyglonax.excavator import ExcavatorAdapter

HOST = "https://cymbion-oybqn.ondigitalocean.app"
INSTANCE = "d09b4882-bc40-42ca-9b33-f7cde4cbc474"
NAME = "test"

config = configparser.ConfigParser()
config.read("config.ini")

host = config["glonax"]["host"]
port = config["glonax"]["port"]

logging.basicConfig(format="%(levelname)s %(message)s", level=logging.INFO)


class TestInputCommand:
    """
    Diagnose the machine
    """

    def __init__(self, host, port):
        self.machine = ExcavatorAdapter(host, port)

    def stop(self):
        self.machine.stop()

    def _probe(self, instance, version, status, name):
        url = f"{HOST}/api/v1/{instance}/probe"
        data = {
            "version": version,
            "status": status,
            "name": name,
        }
        response = requests.post(url, json=data)
        response.raise_for_status()

    def start(self):
        self.machine.start()
        # self.machine.idle()
        # while self.status == self.ConnectionState.CONNECTED:
        logging.info("Starting probe")
        while True:
            time.sleep(60)

            try:
                self._probe(instance=INSTANCE, version=100, status="HEALTHY", name=NAME)
                logging.info("Probe sent successfully")
            except Exception as e:
                logging.error(e)


if __name__ == "__main__":
    program = TestInputCommand(host, port)
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
