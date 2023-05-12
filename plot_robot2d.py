#!/usr/bin/env python3

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pyglonax.excavator import Excavator, ExcavatorAdapter
from pyglonax.util import get_config

matplotlib.use("TkAgg")

config = get_config()

excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])

adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])
adapter.start()
adapter.wait_until_initialized()


fig, ax = plt.subplots(2, 2, figsize=(10, 10))


def animate(i):
    excavator.frame = adapter.encoder["frame"]["angle"]
    excavator.boom = adapter.encoder["boom"]["angle"]
    excavator.arm = adapter.encoder["arm"]["angle"]
    excavator.attachment = adapter.encoder["attachment"]["angle"]

    ax[0][0].clear()
    ax[0][1].clear()
    ax[1][0].clear()

    x = [0]
    y = [0]
    z = [0]

    for joint in excavator.joints:
        effector = excavator.forward_kinematics(joint_name=joint.name)
        x.append(effector[0])
        y.append(effector[1])
        z.append(effector[2])

        ax[0][0].text(
            effector[0],
            effector[2],
            "({:.2f}, {:.2f})".format(effector[0], effector[2]),
        ),
    
        ax[0][1].text(
            effector[1],
            effector[2],
            "({:.2f}, {:.2f})".format(effector[1], effector[2]),
        ),
    
        ax[1][0].text(
            effector[0],
            effector[1],
            "({:.2f}, {:.2f})".format(effector[0], effector[1]),
        ),

    ax[0][0].set_xlim(-12, 12)
    ax[0][0].set_ylim(0, 12)
    
    ax[0][1].set_xlim(-12, 12)
    ax[0][1].set_ylim(0, 12)

    ax[1][0].set_xlim(-12, 12)
    ax[1][0].set_ylim(-12, 12)

    ax[0][0].set_xlabel("X Axis")
    ax[0][0].set_ylabel("Z Axis")
    ax[0][0].plot(x, z, "ro-")

    ax[0][1].set_xlabel("Y Axis")
    ax[0][1].set_ylabel("Z Axis")
    ax[0][1].plot(y, z, "ro-")

    ax[1][0].set_xlabel("Y Axis")
    ax[1][0].set_ylabel("Y Axis")
    ax[1][0].plot(x, y, "ro-")


ani = animation.FuncAnimation(fig, animate, interval=50)
plt.show()

adapter.stop()
