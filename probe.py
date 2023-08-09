#!/usr/bin/env python3

import time
import logging
import requests
import configparser

from pyglonax.excavator import Excavator, ExcavatorAdapter

HOST = "https://cymbion-oybqn.ondigitalocean.app"
VERSION = 101

config = configparser.ConfigParser()
config.read("config.ini")

host = config["glonax"]["host"]
port = config["glonax"]["port"]

robot = config["robot"]["definition_file"]

logging.basicConfig(format="%(levelname)s %(message)s", level=logging.INFO)


class ProbeCommand:
    """
    Diagnose the machine
    """

    def __init__(self, definition_file, host, port):
        self.excavator = Excavator.from_json(file_path=definition_file)
        self.machine = ExcavatorAdapter(host, port)

    def stop(self):
        self.machine.stop()

    def _probe(self, status):
        url = f"{HOST}/api/v1/{self.excavator.instance}/probe"
        data = {
            "version": VERSION,
            "status": status,
            "name": self.excavator.name,
        }

        if "lat" in self.machine.gnss and "long" in self.machine.gnss:
            data["location"] = [self.machine.gnss["lat"], self.machine.gnss["long"]]
        if "altitude" in self.machine.gnss:
            data["altitude"] = self.machine.gnss["altitude"]
        if "speed" in self.machine.gnss:
            data["speed"] = self.machine.gnss["speed"]
        if "satellites" in self.machine.gnss:
            data["satellites"] = self.machine.gnss["satellites"]

        if "memory" in self.machine.vms:
            data["memory"] = self.machine.vms["memory"]
        if "swap" in self.machine.vms:
            data["swap"] = self.machine.vms["swap"]
        if "cpu_1" in self.machine.vms:
            data["cpu_1"] = self.machine.vms["cpu_1"]
        if "cpu_5" in self.machine.vms:
            data["cpu_5"] = self.machine.vms["cpu_5"]
        if "cpu_15" in self.machine.vms:
            data["cpu_15"] = self.machine.vms["cpu_15"]
        if "uptime" in self.machine.vms:
            data["uptime"] = self.machine.vms["uptime"]

        response = requests.post(url, json=data)
        response.raise_for_status()

    def start(self):
        self.machine.start()
        logging.info("Starting probe")

        while not self.machine.is_terminated():
            time.sleep(60)

            try:
                self._probe(status="HEALTHY")
                logging.info("Probe sent successfully")
            except Exception as e:
                logging.error(e)


if __name__ == "__main__":
    program = ProbeCommand(robot, host, port)
    try:
        program.start()
    except KeyboardInterrupt:
        program.stop()
