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


fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(projection="3d")


def animate(i):
    excavator.frame = adapter.encoder["frame"]["angle"]
    excavator.boom = adapter.encoder["boom"]["angle"]
    excavator.arm = adapter.encoder["arm"]["angle"]
    excavator.attachment = adapter.encoder["attachment"]["angle"]

    ax.clear()

    x = [0]
    y = [0]
    z = [0]

    for joint in excavator.joints:
        effector = excavator.forward_kinematics(joint_name=joint.name)
        x.append(effector[0])
        y.append(effector[1])
        z.append(effector[2])

        ax.text(
            effector[0],
            effector[1],
            effector[2],
            "({:.2f}, {:.2f}, {:.2f})".format(effector[0], effector[1], effector[2]),
        ),

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 10)

    ax.set_xlabel("X Axis")
    ax.set_ylabel("Y Axis")
    ax.set_zlabel("Z Axis")
    ax.plot(x, y, z, "ro-")


ani = animation.FuncAnimation(fig, animate, interval=100)
plt.show()

adapter.stop()
