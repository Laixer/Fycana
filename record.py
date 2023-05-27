#!/usr/bin/env python3

import os
import sys
import json
import traceback


from pyglonax.excavator import Excavator, ExcavatorAdapter
from pyglonax.util import get_config, format_euler_tuple


config = get_config()


class Recorder:
    def __init__(self, definition_file, host):
        self.excavator = Excavator.from_urdf(file_path=definition_file)
        self.adapter = ExcavatorAdapter(host=host)
        self.adapter.on_signal_update(self._update_signal)
        self.program = []

    def _update_signal(self, signal):
        if "frame" in self.adapter.encoder:
            self.excavator.frame = self.adapter.encoder["frame"]["angle"]
        if "boom" in self.adapter.encoder:
            self.excavator.boom = self.adapter.encoder["boom"]["angle"]
        if "arm" in self.adapter.encoder:
            self.excavator.arm = self.adapter.encoder["arm"]["angle"]
        if "attachment" in self.adapter.encoder:
            self.excavator.attachment = self.adapter.encoder["attachment"]["angle"]

    def start(self):
        self.adapter.start()
        self.adapter.wait_until_initialized()

    def record(self):
        effector = self.excavator.forward_kinematics2(joint_name="attachment_joint")
        effector_array = [round(item, 3) for item in effector.tolist()]
        self.program.append(effector_array)
        return effector_array

    def stop(self):
        self.adapter.stop()


if __name__ == "__main__":
    args = sys.argv[1:]

    if len(args) < 1:
        print("Usage: python3 record.py <program.json>")
        sys.exit(1)

    print("Recording model:", args[0])

    filename = args[0]

    recorder = Recorder(
        definition_file=config["ROBOT_DEFINITION"],
        host=config["GLONAX_HOST"],
    )
    try:
        recorder.start()

        while True:
            input("Press Enter to record a step, press ctrl-c to stop...")

            effector = recorder.record()
            print("Recorded End effector:", format_euler_tuple(effector))

    except KeyboardInterrupt:
        pass
    except Exception as e:
        traceback.print_exception(type(e), e, e.__traceback__)
    finally:
        recorder.stop()

        with open(filename, "w") as outfile:
            json.dump(recorder.program, outfile)

            print()
            print(f"Program saved to {filename}")
