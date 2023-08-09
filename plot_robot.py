#!/usr/bin/env python3

import configparser
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter


matplotlib.use("TkAgg")


config = configparser.ConfigParser()
config.read("config.ini")

host = config["glonax"]["host"]
port = config["glonax"]["port"]


excavator = Excavator.from_json(file_path=config["robot"]["definition_file"])

adapter = ExcavatorAdapter(host, port)

articulation_chain = excavator.get_chain_by_name("articulation_arm")
kinematic_chain = excavator.get_chain_by_name("kinematic_arm")


def _update_signal(signal):
    for joint in articulation_chain.joints:
        if joint.name in adapter.encoder:
            excavator.set_position_state(
                joint.name, adapter.encoder[joint.name]["angle"]
            )


adapter.on_signal_update(_update_signal)
adapter.start()
adapter.wait_until_initialized()

fig, ax = plt.subplots(1, 1, figsize=(10, 10), subplot_kw=dict(projection="3d"))


def animate(i):
    ax.clear()

    x = [0]
    y = [0]
    z = [0]

    for effector in kinematic_chain.opspace_forward_kinematics(result_only=False):
        x.append(effector[0])
        y.append(effector[1])
        z.append(effector[2])

        ax.text(
            effector[0],
            effector[1],
            effector[2],
            "({:.2f}, {:.2f}, {:.2f}) [{:.2f}, {:.2f}, {:.2f}]".format(
                effector[0],
                effector[1],
                effector[2],
                np.rad2deg(effector[3]),
                np.rad2deg(effector[4]),
                np.rad2deg(effector[5]),
            ),
        ),

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 10)

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.plot(x, y, z, "ro-")


ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()

adapter.stop()
