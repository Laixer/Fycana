import os
import sys
import time
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter
from pyglonax.util import get_config

config = get_config()

np.set_printoptions(formatter={"float": lambda x: "{0:0.2f}".format(x)})

program = np.load(file="model/default_trainnig_v1.npy")
excavator = Excavator.from_urdf(file_path=config["ROBOT_DEFINITION"])

print(excavator)

# sys.exit(0)

adapter = ExcavatorAdapter(host=config["GLONAX_HOST"])
# adapter.stop()
# adapter.idle()

# excavator.plot_robot()

# for x in adapter.body.edges(data=True):
# print(x)

adapter.start()
adapter.wait_until_initialized()

print("Machine initialized")

excavator.frame = adapter.encoder["frame"]["angle"]
excavator.boom = adapter.encoder["boom"]["angle"]
excavator.arm = adapter.encoder["arm"]["angle"]

effector = excavator.forward_kinematics()
print("End effector:", effector)

# adapter.stop()
# sys.exit(0)

# print(excavator)

print("Program:", program)

# sys.exit(0)
print("Starting program")

for target in program:
    effector = excavator.forward_kinematics()
    print("")
    print("End effector:", effector)
    print("Target:", target)

    excavator.inverse_kinematics(target)

    while True:
        if excavator.is_objective_reached():
            print("Objective reached")
            input("Press Enter to continue...")
            break
        else:
            print("Objective not reached")

        e = excavator.get_position_error()[0]

        print("Error:", e)

        g = excavator.position_state[0] + e

        print("Correction:", g)

        excavator.frame = g[1]
        excavator.boom = g[2]
        excavator.arm = g[3]
        excavator.attachment = g[4]

        print("")

        time.sleep(0.5)
