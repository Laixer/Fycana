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

    x = [0]
    y = [0]
    z = [0]

    effector = excavator.forward_kinematics(joint_name="frame_joint")
    x.append(effector[0])
    y.append(effector[1])
    z.append(effector[2])

    boom_point = excavator.forward_kinematics(joint_name="boom_joint")
    x.append(boom_point[0])
    y.append(boom_point[1])
    z.append(boom_point[2])

    ax.clear()

    ax.text(
        boom_point[0],
        boom_point[1],
        boom_point[2],
        "({:.2f}, {:.2f}, {:.2f})".format(boom_point[0], boom_point[1], boom_point[2]),
    ),

    arm_point = excavator.forward_kinematics(joint_name="arm_joint")
    x.append(arm_point[0])
    y.append(arm_point[1])
    z.append(arm_point[2])

    ax.text(
        arm_point[0],
        arm_point[1],
        arm_point[2],
        "({:.2f}, {:.2f}, {:.2f})".format(arm_point[0], arm_point[1], arm_point[2]),
    ),

    effector = excavator.forward_kinematics()
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
