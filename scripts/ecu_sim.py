import grpc
import time
import logging
import pybullet as b
import pybullet_data
import threading
from concurrent import futures
from xbox360controller import Xbox360Controller

import ecu_pb2
import ecu_pb2_grpc

logging.basicConfig(format='%(levelname)s %(message)s', level=logging.INFO)


def get_joint_by_name(uid, name):
    nums = b.getNumJoints(uid)
    for i in range(nums):
        info = b.getJointInfo(uid, i)
        link_name = info[1].decode("utf-8")
        if link_name == name:
            return info[0]


def load_env():
    b.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Load the floor plane
    ground_id = b.loadURDF("plane.urdf")

    # Load the excavator robot
    robot_id = b.loadURDF("./urdf/volvo_ec240cl.urdf", flags=b.URDF_USE_SELF_COLLISION)

    # Load cone obstacle
    # cone_id = b.loadURDF("cone.urdf")

    # Load cube obstacle
    cube1_id = b.loadURDF(
        "cube.urdf",
        [3, 3.5, 0.5],
        # useFixedBase=True
    )

    # Store body indices in a dict with more convenient key names
    bodies = {
        "robot": robot_id,
        "ground": ground_id,
        "cube1": cube1_id,
        # "cone1": cone_id,
    }

    return bodies


client = b.connect(b.GUI)
bodies = load_env()

b.setGravity(0, 0, -9.81)
b.setTimeStep(0.001)
# b.setRealTimeSimulation(1)

excavator = bodies["robot"]

frame_joint = get_joint_by_name(excavator, "frame_joint")
boom_joint = get_joint_by_name(excavator, "boom_joint")
arm_joint = get_joint_by_name(excavator, "arm_joint")
attachment_joint = get_joint_by_name(excavator, "attachment_joint")

b.setJointMotorControl2(excavator, frame_joint, b.VELOCITY_CONTROL, force=0)
b.setJointMotorControl2(excavator, boom_joint, b.VELOCITY_CONTROL, force=0)
b.setJointMotorControl2(excavator, arm_joint, b.VELOCITY_CONTROL, force=0)
b.setJointMotorControl2(excavator, attachment_joint, b.VELOCITY_CONTROL, force=0)

frame_control_val = None
boom_control_val = None
arm_control_val = None
attachment_control_val = None


def run_sim():
    while True:
        b.stepSimulation()

        if boom_control_val:
            b_act, _, _, _ = b.getJointState(excavator, boom_joint)
            next_pos = b_act-(boom_control_val/100_000)
            b.setJointMotorControl2(excavator, boom_joint, controlMode=b.POSITION_CONTROL, targetPosition=next_pos)

        if frame_control_val:
            f_act, _, _, _ = b.getJointState(excavator, frame_joint)
            next_pos = f_act-(frame_control_val/100_000)
            b.setJointMotorControl2(excavator, frame_joint, controlMode=b.POSITION_CONTROL, targetPosition=next_pos)

        if arm_control_val:
            a_act, _, _, _ = b.getJointState(excavator, arm_joint)
            next_pos = a_act+(arm_control_val/100_000)
            b.setJointMotorControl2(excavator, arm_joint, controlMode=b.POSITION_CONTROL, targetPosition=next_pos)

        if attachment_control_val:
            u_act, _, _, _ = b.getJointState(excavator, attachment_joint)
            next_pos = u_act+(attachment_control_val/100_000)
            b.setJointMotorControl2(excavator, attachment_joint, controlMode=b.POSITION_CONTROL, targetPosition=next_pos)

        time.sleep(1/120)


x = threading.Thread(target=run_sim)
x.start()


class MotionServicer(ecu_pb2_grpc.MotionServicer):
    def Act(self, request, context):
        global frame_control_val, boom_control_val, arm_control_val, attachment_control_val

        match request.type:
            case 0:  # stopall
                frame_control_val = None
                boom_control_val = None
                arm_control_val = None
                attachment_control_val = None
            case 1:  # resumeall
                pass
            case 2:  # change
                for changeset in request.changes:
                    print(f"Le x {changeset.actuator} => {changeset.value}")
                    match changeset.actuator:
                        case 0:  # boom
                            boom_control_val = changeset.value
                        case 1:  # frame
                            frame_control_val = changeset.value
                        case 2:  # TrackRight
                            pass
                        case 3:  # TrackLeft
                            pass
                        case 4:  # arm
                            arm_control_val = changeset.value
                        case 5:  # attachment
                            attachment_control_val = changeset.value

        return ecu_pb2.Empty()


class SignalServicer(ecu_pb2_grpc.SignalServicer):
    def Listen(self, request, context):
        while True:
            b_act, _, _, _ = b.getJointState(excavator, boom_joint)
            enc = ecu_pb2.EncoderSet(id=0, position=0, speed=0, angle=int(b_act * 1000))
            yield enc

            f_act, _, _, _ = b.getJointState(excavator, frame_joint)
            enc = ecu_pb2.EncoderSet(id=1, position=0, speed=0, angle=int(f_act * 1000))
            yield enc

            a_act, _, _, _ = b.getJointState(excavator, arm_joint)
            enc = ecu_pb2.EncoderSet(id=4, position=0, speed=0, angle=int(a_act * 1000))
            yield enc

            u_act, _, _, _ = b.getJointState(excavator, attachment_joint)
            enc = ecu_pb2.EncoderSet(id=5, position=0, speed=0, angle=int(u_act * 1000))
            yield enc
            time.sleep(0.05)


server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))

ecu_pb2_grpc.add_MotionServicer_to_server(MotionServicer(), server)
ecu_pb2_grpc.add_SignalServicer_to_server(SignalServicer(), server)

server.add_insecure_port('[::]:50051')
server.start()
server.wait_for_termination()
