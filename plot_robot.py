import os
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from pyglonax.excavator import Excavator, ExcavatorAdapter

matplotlib.use("TkAgg")

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(projection="3d")


def from_local_path(file, sub_dir=None):
    script_dir = os.path.dirname(__file__)
    if sub_dir is None:
        rel_path = file
    else:
        rel_path = os.path.join(sub_dir, file)
    abs_file_path = os.path.join(script_dir, rel_path)
    return abs_file_path


excavator = Excavator.from_urdf(from_local_path("volvo_ec240cl.urdf", "urdf"))

adapter = ExcavatorAdapter()
adapter.start()
adapter.wait_until_initialized()


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

    effector = excavator.forward_kinematics(joint_name="boom_joint")
    x.append(effector[0])
    y.append(effector[1])
    z.append(effector[2])

    effector = excavator.forward_kinematics(joint_name="arm_joint")
    x.append(effector[0])
    y.append(effector[1])
    z.append(effector[2])

    effector = excavator.forward_kinematics()
    x.append(effector[0])
    y.append(effector[1])
    z.append(effector[2])

    ax.clear()

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
