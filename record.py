#!/usr/bin/env python3

import os
import sys
import time
from threading import Thread, Event
import json
import numpy as np


from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.util import get_config
from pyglonax.alg import shortest_rotation

config = get_config()

# np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})


def format_angle(value):
    return "{:.2f}°".format(np.rad2deg(value))


def format_coord(value):
    return "{:.2f}".format(value)


def format_euler_tuple(effector):
    return "({}, {}, {}) [{}, {}, {}]".format(
        format_coord(effector[0]),
        format_coord(effector[1]),
        format_coord(effector[2]),
        format_angle(effector[3]),
        format_angle(effector[4]),
        format_angle(effector[5]),
    )


tolerance = float(config["ROBOT_KIN_TOL"])

excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])
adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])

program = []


class Worker(Thread):
    def __init__(self, event, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.event = event

    def run(self) -> None:
        while not self.event.is_set():
            excavator.frame = adapter.encoder["frame"]["angle"]
            excavator.boom = adapter.encoder["boom"]["angle"]
            excavator.arm = adapter.encoder["arm"]["angle"]
            excavator.attachment = adapter.encoder["attachment"]["angle"]
            time.sleep(0.1)


event = Event()
thread = Worker(event)

try:
    adapter.start()
    adapter.wait_until_initialized()

    print("Machine initialized")

    thread.start()

    while True:
        input("Press Enter to record a step, press ctrl-c to stop...")

        effector = excavator.forward_kinematics2(joint_name="attachment_joint")
        print("Recorded End effector:", format_euler_tuple(effector))

        effector_array = [round(item, 3) for item in effector.tolist()]

        program.append(effector_array)
except KeyboardInterrupt:
    event.set()
    thread.join()
    adapter.disable_motion()
    adapter.stop()

    import random
    import string

    def generate_random_filename():
        length = random.randint(5, 10)
        return "model_" + (
            "".join(
                random.choice(string.ascii_lowercase + string.digits)
                for _ in range(length)
            )
            + ".json"
        )

    filename = generate_random_filename()

    with open(f"model/{filename}", "w") as outfile:
        json.dump(program, outfile)

        print()
        print(f"Program saved to {filename}")

    sys.exit(0)
