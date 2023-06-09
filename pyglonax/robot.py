import json
import numpy as np
import scipy as sp
from scipy import optimize

from . import geom, util, motion
from .util import format_euler_tuple


def calculate_forward_kinematics(joint_parameters, joints):
    if len(joints) != len(joint_parameters):
        raise ValueError(
            f"joint_parameters has length {len(joint_parameters)} but robot has {len(joints)} joints"
        )

    euler_matrix = np.identity(4)
    rotation_matrix = np.array([0, 0, 0])
    frame_cumulative = np.empty((0, 6))
    for joint, joint_parameter in zip(joints, joint_parameters):
        joint_rotation = np.array([0, 0, 0])
        if joint.origin_translation is not None:
            euler_matrix = euler_matrix.dot(
                geom.Geometry.translation_matrix(*joint.origin_translation)
            )
        if joint.origin_orientation is not None:
            euler_matrix = euler_matrix.dot(
                geom.Geometry.rotation_matrix(*joint.origin_orientation[1:])
            )
            joint_rotation = joint.origin_orientation
        if joint.rotation is not None:
            rotation = joint.rotation * joint_parameter
            euler_matrix = euler_matrix.dot(
                geom.Geometry.rotation_matrix(*rotation[1:])
            )
            joint_rotation = joint_rotation + rotation
        if joint.translation is not None:
            translation = joint.translation * joint_parameter
            euler_matrix = euler_matrix.dot(
                geom.Geometry.translation_matrix(*translation)
            )

        rotation_matrix = rotation_matrix + joint_rotation
        coord_rot = np.concatenate((euler_matrix[:3, 3], rotation_matrix), axis=0)
        frame_cumulative = np.append(frame_cumulative, np.array([coord_rot]), axis=0)

    return frame_cumulative


class Joint:
    def __init__(
        self,
        name: str,
        type="fixed",
        origin_translation=None,
        origin_orientation=None,
        rotation=None,
        translation=None,
        bounds=np.array([-np.inf, np.inf]),
        motion_profile=None,
    ):
        self.name = name
        self.type = type

        self.origin_translation = origin_translation
        self.origin_orientation = origin_orientation
        self.rotation = rotation
        self.translation = translation
        self.bounds = bounds
        self.motion_profile = motion_profile

    @property
    def lower_bound(self):
        """Return the lower bound of the joint"""
        return self.bounds[0]

    @property
    def upper_bound(self):
        """Return the upper bound of the joint"""
        return self.bounds[1]

    @property
    def domain(self):
        """Return the domain of the joint"""
        return np.diff(self.bounds)[0]

    def default(self):
        """Return the default value of the joint"""
        return (
            0.0
            if np.isinf(self.lower_bound) or self.is_within_bounds(0.0)
            else self.lower_bound
        )

    def normalize(self, value):
        """Normalize the given value to the domain of the joint"""
        return (value - self.lower_bound) / self.domain

    def is_within_bounds(self, value):
        """Check if the given value is within the bounds of the joint"""
        return self.lower_bound <= value <= self.upper_bound

    def clip(self, value):
        """Clip the given value to the bounds of the joint"""
        return np.clip(value, self.lower_bound, self.upper_bound)

    def cycle(self, value, domain=None):
        """Cycle the given value to the bounds of the joint"""
        return np.mod(value, self.domain if domain is None else domain)

    def reduce(self, value):
        """Reduce the given value to the bounds of the joint"""
        if self.type == "continuous":
            return self.cycle(value, 2 * np.pi)
        elif self.type == "revolute":
            return self.clip(value)
        return value

    def __str__(self):
        origin_translation = self.origin_translation
        if self.origin_translation is None:
            origin_translation = np.array([0.0, 0.0, 0.0])

        origin_orientation = self.origin_orientation
        if self.origin_orientation is None:
            origin_orientation = np.array([0.0, 0.0, 0.0])

        translation = self.translation
        if self.translation is None:
            translation = np.array([0.0, 0.0, 0.0])

        rotation = self.rotation
        if self.rotation is None:
            rotation = np.array([0.0, 0.0, 0.0])

        return f"Joint={self.name}, Type={self.type}, Origin={format_euler_tuple(np.concatenate((origin_translation, origin_orientation)))}, Transformation={format_euler_tuple(np.concatenate((translation, rotation)))}, Bounds={self.bounds}"


class Chain:
    def __init__(self, robot, name, joints):
        self.robot = robot
        self.name = name
        self.joints = joints
        self.position_state = np.array([joint.default() for joint in self.joints])

    def reset(self):
        """Reset joint positions to default"""
        self.position_state = np.array([joint.default() for joint in self.joints])

    # TODO: Rename physical to opspace
    def physical_state(self):
        joint_idx_list = []
        for joint in self.joints:
            joint_idx = self.robot._get_joint_index_by_name(joint.name)
            joint_idx_list.append(joint_idx)
        return self.robot.get_position_state_actual()[joint_idx_list]

    def projected_state(self):
        return self.position_state

    def error(self):
        """Return the error between the physical and projected state"""
        dual_state = np.array([self.physical_state(), self.position_state])
        return np.diff(dual_state, axis=0)[0]

    def is_objective_reached(self, tolerance=0.01):
        """Check if the objective is reached"""
        return np.all(np.abs(self.error()) < tolerance)

    def forward_kinematics(self, joint_parameters=None, result_only=True) -> np.ndarray:
        frame = calculate_forward_kinematics(joint_parameters, self.joints)
        if result_only:
            return frame[-1]
        return frame

    def opspace_forward_kinematics(self, result_only=True) -> np.ndarray:
        return self.forward_kinematics(self.physical_state(), result_only)

    def projected_forward_kinematics(self, result_only=True) -> np.ndarray:
        return self.forward_kinematics(self.projected_state(), result_only)

    def inverse_kinematics(self, target) -> np.ndarray:
        if target.size != 3:
            raise ValueError(f"point has size {target.size} but should be 3")

        frame_matrix = np.identity(4)
        frame_matrix[:3, 3] = target

        target = frame_matrix[:3, 3]

        # TODO: The lower bound must also include the origin of the joint
        lb = [joint.lower_bound for joint in self.joints]
        ub = [joint.upper_bound for joint in self.joints]

        def eurler_error(x):
            frame = self.forward_kinematics(x, result_only=True)
            return frame[:3] - target

        res = optimize.least_squares(
            eurler_error,
            self.physical_state(),
            bounds=(lb, ub),
            ftol=None,
        )
        if not res.success:
            raise ValueError("Could not find inverse kinematics solution")

        for idx, joint in enumerate(self.joints):
            self.position_state[idx] = joint.reduce(res.x[idx])

        return res

    def __str__(self):
        s = f"Chain={self.name}, Joints={[joint.name for joint in self.joints]}"
        s += f"\n    PhysicalEffector={format_euler_tuple(self.opspace_forward_kinematics())}"
        s += f"\n    ProjectedEffector={format_euler_tuple(self.projected_forward_kinematics())}"
        return s


class Device:
    def __init__(self, name, id, type):
        self.name = name
        self.id = id
        self.type = type

    def __str__(self):
        return f"Device={self.name}, Id={self.id}, Type={self.type}"


class Robot:
    def __init__(self, name, model=None):
        self.name = name
        self.model = model
        self.joints = list()  # TODO: Replace with graph
        self.chains = list()
        self.devices = list()
        self.position_state = np.zeros((1, 0))

    @classmethod
    def from_json(cls, file_path):
        """Create a robot from a JSON object"""
        f = open(file_path)
        definition_file = json.load(f)
        f.close()
        if definition_file["version"] != 1:
            raise ValueError("Invalid JSON definition file version")

        name = definition_file["name"]
        model = definition_file["model"]
        robot = cls(name, model)
        devices = definition_file["devices"]
        for device in devices:
            device_name = device["name"]
            device_id = device["id"]
            device_type = device["type"]
            robot.devices.append(Device(device_name, device_id, device_type))
        body = definition_file["body"]
        chains = body["chain"]
        joints = body["joint"]
        for joint in joints:
            joint_name = joint["name"]
            joint_type = joint["type"]
            joint_origin_translation = None
            joint_origin_orientation = None
            if "origin" in joint:
                if "translation" in joint["origin"]:
                    joint_origin_translation = joint["origin"]["translation"]
                if "orientation" in joint["origin"]:
                    joint_origin_orientation = joint["origin"]["orientation"]
            joint_rotation = None
            joint_translation = None
            if "axis" in joint:
                match joint_type:
                    case "revolute" | "continuous":
                        joint_rotation = np.array(joint["axis"], dtype=np.float64)
                    case "prismatic":
                        joint_translation = np.array(joint["axis"], dtype=np.float64)
            bounds = np.array([-np.inf, np.inf])
            if "limit" in joint:
                if joint_type == "revolute" or joint_type == "prismatic":
                    if "lower" in joint["limit"]:
                        bounds[0] = joint["limit"]["lower"]
                    if "upper" in joint["limit"]:
                        bounds[1] = joint["limit"]["upper"]
            motion_profile = None
            if "power" in joint:
                power_scale = 0
                power_offset = 0
                power_tolerance = 0.0
                power_inverse = False
                if "scale" in joint["power"]:
                    power_scale = int(joint["power"]["scale"])
                if "offset" in joint["power"]:
                    power_offset = int(joint["power"]["offset"])
                if "tolerance" in joint["power"]:
                    power_tolerance = float(joint["power"]["tolerance"])
                if "inverse" in joint["power"]:
                    if joint["power"]["inverse"]:
                        power_inverse = True
                motion_profile = motion.MotionProfile(
                    power_scale, power_offset, power_tolerance, power_inverse
                )

            joint = Joint(
                name=joint_name,
                type=joint_type,
                origin_translation=joint_origin_translation,
                origin_orientation=joint_origin_orientation,
                rotation=joint_rotation,
                translation=joint_translation,
                bounds=bounds,
                motion_profile=motion_profile,
            )
            robot.joints.append(joint)
        for chain in chains:
            chain_name = chain["name"]
            chan_order = chain["order"]

            joints = []
            for joint_link in chan_order:
                joint = robot.get_joint_by_name(joint_link)
                joints.append(joint)
            robot.chains.append(Chain(robot, chain_name, joints))

        robot.position_state = np.array([joint.default() for joint in robot.joints])
        return robot

    def get_joint_by_name(self, name) -> Joint:
        """Get joint by name"""
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError("Joint not found")

    def get_chain_by_name(self, name) -> Chain:
        """Get chain by name"""
        for chain in self.chains:
            if chain.name == name:
                return chain
        raise ValueError("Chain not found")

    def _get_joint_index_by_name(self, name) -> int:
        """Get joint index by name"""
        for idx, joint in enumerate(self.joints):
            if joint.name == name:
                return idx
        raise ValueError("Joint not found")

    def get_position_state(self, name) -> float:
        """Get joint position by name"""
        idx = self._get_joint_index_by_name(name)
        return self.position_state[idx]

    def set_position_state(self, name, value):
        """Set joint position by name"""
        idx = self._get_joint_index_by_name(name)
        if self.joints[idx].type == "fixed":
            raise ValueError(f"Joint {name} is fixed")
        self.position_state[idx] = value

    def get_position_state_actual(self):
        return self.position_state

    # TODO: Clean up
    def __str__(self) -> str:
        s = f"Robot={self.name}"
        for device in self.devices:
            s += f"\n  {device}"
        if self.model is not None:
            s += f", Model={self.model}"
        s += f", Chains={len(self.chains)}, Joints={len(self.joints)}"
        for chain in self.chains:
            s += f"\n  {chain}"
        for joint in self.joints:
            s += f"\n  {joint}"
        s += "\n"
        return s
