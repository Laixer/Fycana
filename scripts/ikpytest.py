import matplotlib.pyplot as plt
import matplotlib
import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import sys

f = "/home/yorick/Projects/fycana/urdf/volvo_ec240cl.urdf"
my_chain = ikpy.chain.Chain.from_urdf_file(f, base_elements=["base"], active_links_mask=[False,  False,  True,  True,  True, False])

# links = ikpy.urdf.URDF.get_urdf_parameters(f, base_elements=["base"])
# print(links)
print(my_chain)
sys.exit(0)

target_position = [5.21, 0, 1]

# real_frame = my_chain.forward_kinematics([0, 0, np.deg2rad(30), 0, 0, 0])
# real_frame = my_chain.forward_kinematics([0, 0, 0, 0, 0, 0])
# print(real_frame)

inv = my_chain.inverse_kinematics(target_position)
print("The angles of each joints are : ")
for x in inv:
    print(np.rad2deg(x))

# real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_position))
# print("Computed position vector : %s, original position vector : %s" % (real_frame[:3, 3], target_position))


matplotlib.use('TkAgg')
fig, ax = plot_utils.init_3d_figure()
# my_chain.plot([0,  0,   0,  -np.deg2rad(30),  np.deg2rad(90+30), 0], ax, target=target_position)
my_chain.plot(inv, ax, target=target_position)
plt.xlim(-1.1, 1.1)
plt.ylim(-1.1, 1.1)
plt.show()
