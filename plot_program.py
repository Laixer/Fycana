import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.use("TkAgg")

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(projection="3d")


program = np.load(file="model/default_trainnig_v1.npy")

ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)
ax.set_zlim(0, 10)

ax.set_xlabel("X Axis")
ax.set_ylabel("Y Axis")
ax.set_zlabel("Z Axis")

last_target = None
for target in program:
    if last_target is not None:
        ax.plot(
            [last_target[0], target[0]],
            [last_target[1], target[1]],
            [last_target[2], target[2]],
            "ro-",
        )
        plt.pause(2.5)
    last_target = target


plt.show()
