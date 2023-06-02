#!/usr/bin/env python3

import os
import sys
import time
import traceback
import numpy as np

from pyglonax.excavator import Excavator, ExcavatorAdapter, ExcavatorActuator
from pyglonax.motion import MotionProfile
from pyglonax.util import format_euler_tuple, format_angle
from pyglonax.alg import shortest_rotation

np.set_printoptions(formatter={"float": lambda x: "{0:0.3f}".format(x)})

# Set the absolute tolerance for the position error
tolerance = 0.01

motion_profile_slew = MotionProfile(10_000, 12_000, tolerance, False)
motion_profile_boom = MotionProfile(15_000, 12_000, tolerance, True)
motion_profile_arm = MotionProfile(15_000, 12_000, tolerance, False)
motion_profile_attachment = MotionProfile(15_000, 12_000, tolerance, False)

excavator = Excavator.from_json(file_path="robot/volvo_ec240cl.json")

print(excavator)

# sys.exit(0)

adapter = ExcavatorAdapter(host="localhost:50051")

# excavator.plot_robot()

# for x in adapter.body.edges(data=True):
# print(x)

adapter.start()
adapter.wait_until_initialized()

print("Machine initialized")


excavator.set_position_state("frame", adapter.encoder["frame"]["angle"])
excavator.set_position_state("boo", adapter.encoder["boom"]["angle"])
excavator.set_position_state("arm", adapter.encoder["arm"]["angle"])
excavator.set_position_state("attachment", adapter.encoder["attachment"]["angle"])

target = np.array([4.09, 0.89, -2.32])

# excavator.inverse_kinematics(target)

# print("\n")

# effector = excavator.forward_kinematics3(chain_name="articulation_arm")
# print("End effector1:", format_euler_tuple(effector))

# print("Target:", target)

# joint_idx_list = []
# chain = excavator.get_chain_by_name("articulation_arm")
# for joint in chain.joints:
#     joint_idx = excavator._get_joint_index_by_name(joint.name)
#     joint_idx_list.append(joint_idx)

# proj = excavator.get_position_state_projected()[joint_idx_list]
# # frame = excavator._calculate_forward_kinematics(proj, joint_name="attachment")[-1][:3]
# frame = excavator.forward_kinematics3(
#     chain_name="articulation_arm", joint_parameters=proj
# )[:3]

# print("Projected Euler:", frame)
# print("Projected angle:", proj)

print()

chain_arm = excavator.get_chain_by_name("articulation_arm")

forward_kinematics = chain_arm.opspace_forward_kinematics()
print("PRE Chain arm opsp.FK :", format_euler_tuple(forward_kinematics))
forward_kinematics_projected = chain_arm.projected_forward_kinematics()
print("PRE Chain arm proj.FK :", format_euler_tuple(forward_kinematics_projected))

target = np.array([5.56, 0.00, 1.65])
chain_arm.inverse_kinematics(target)

forward_kinematics = chain_arm.opspace_forward_kinematics()
print("POS Chain arm opsp.FK :", format_euler_tuple(forward_kinematics))
forward_kinematics_projected = chain_arm.projected_forward_kinematics()
print("POS Chain arm proj.FK :", format_euler_tuple(forward_kinematics_projected))


adapter.disable_motion()
adapter.stop()
sys.exit(0)
