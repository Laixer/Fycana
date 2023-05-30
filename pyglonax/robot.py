import numpy as np
import networkx as nx
import scipy as sp
from scipy import optimize

from xml.etree import ElementTree as ET

from . import geom, util


class Link:
    def __init__(self, name):
        self.name = name

    def __str__(self):
        return f"Link={self.name}"


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
    ):
        self.name = name
        self.type = type

        self.origin_translation = origin_translation
        self.origin_orientation = origin_orientation
        self.rotation = rotation
        self.translation = translation
        self.bounds = bounds

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
            print(f"cycle: {value}")
            return self.cycle(value, 2 * np.pi)
        elif self.type == "revolute":
            print(f"clip: {value}")
            return self.clip(value)
        return value

    def __str__(self):
        return f"Joint={self.name}, Type={self.type}, OriginTranslation={util.numpy3d_to_string(self.origin_translation)}, OriginOrientation={util.numpy3d_to_string(self.origin_orientation)}, Rotation={util.numpy3d_to_string(self.rotation)}, Translation={util.numpy3d_to_string(self.translation)}, Bounds={self.bounds}"


class Robot:
    def __init__(self, name, model=None):
        self.name = name
        self.model = model
        self.links = list()  # TODO: Replace with graph
        self.joints = list()  # TODO: Replace with graph
        self.joint_mask = np.array([])
        self.position_state = np.zeros((2, 0))
        self.body = nx.DiGraph()

    def add_joint(self, joint, parent=None, child=None):
        """Add a joint to the robot"""
        self.joints.append(joint)

        # TODO: Rather go with .append()
        # Update the position state with an new column for the joint
        self.position_state = np.c_[
            self.position_state,
            np.full((self.position_state.shape[0], 1), joint.default()),
        ]
        # self.joint_mask = np.append(
        #     self.joint_mask, np.full((self.joint_mask.shape[0], 1), 14), axis=1
        # )
        default_mask = 1
        self.joint_mask = np.append(self.joint_mask, [default_mask], axis=0)

        if parent is not None and child is not None:
            self.body.add_edge(parent, child, joint=joint)

    @classmethod
    def from_urdf(cls, file_path):
        """Create a robot from an URDF file"""

        tree_xml = ET.parse(file_path)
        urdf_root = tree_xml.getroot()

        # Fetch the name of the robot
        name = urdf_root.attrib.get("name")

        # The URDF specification does not include a model attribute for the robot
        # but we can use it to specify the robot model.
        model = urdf_root.attrib.get("model")

        # Create the robot
        robot = cls(name, model)
        robot._urdf_root = urdf_root

        # Build the list of joints of this robot
        for idx, elem_joint in enumerate(urdf_root.findall("joint")):
            attrib = elem_joint.attrib
            joint_name = attrib.get("name", idx)
            joint_type = attrib.get("type", idx)

            # Find the parent link, if any
            parent_tag = elem_joint.find("parent")
            parent = parent_tag.attrib["link"]

            # Find the child link, if any
            child_tag = elem_joint.find("child")
            child = child_tag.attrib["link"]

            origin_translation = None
            origin_orientation = None
            rotation = None
            translation = None
            bounds = np.array(
                [-np.inf, np.inf]
            )  # TODO: Maybe set to 0 for fixed joints

            # Find the translation and rotation of the joint origin
            elem_origin = elem_joint.find("origin")
            if elem_origin is not None:
                if "xyz" in elem_origin.attrib.keys():
                    nd = np.array(
                        [float(data) for data in elem_origin.attrib.get("xyz").split()]
                    )
                    origin_translation = nd

                if "rpy" in elem_origin.attrib.keys():
                    nd = np.array(
                        [float(data) for data in elem_origin.attrib.get("rpy").split()]
                    )
                    origin_orientation = nd

            # Find the axis of rotation or translation
            elem_axis = elem_joint.find("axis")
            if elem_axis is not None:
                match joint_type:
                    case "revolute" | "continuous":
                        nd = np.array(
                            [float(x) for x in elem_axis.attrib.get("xyz").split()]
                        )
                        rotation = nd
                    case "prismatic":
                        nd = np.array(
                            [float(x) for x in elem_axis.attrib.get("xyz").split()]
                        )
                        translation = nd

            # Find the joint limits
            elem_limit = elem_joint.find("limit")
            if elem_limit is not None:
                if joint_type == "revolute" or joint_type == "prismatic":
                    if "lower" in elem_limit.attrib:
                        bounds[0] = elem_limit.attrib["lower"]
                    if "upper" in elem_limit.attrib:
                        bounds[1] = elem_limit.attrib["upper"]

            joint = Joint(
                name=joint_name,
                type=joint_type,
                origin_translation=origin_translation,
                origin_orientation=origin_orientation,
                rotation=rotation,
                translation=translation,
                bounds=bounds,
            )
            robot.add_joint(joint, parent, child)

        # Build the list of links of this robot
        for idx, elem_link in enumerate(urdf_root.findall("link")):
            attrib = elem_link.attrib
            link_name = attrib.get("name", idx)

            link = Link(link_name)
            robot.links.append(link)

            # Add all nodes that are not already in the graph
            robot.body.add_node(link_name)

        return robot

    def get_joint_by_name(self, name) -> Joint:
        """Get joint by name"""
        for joint in self.joints:
            if joint.name == name:
                return joint
        raise ValueError("Joint not found")

    def _get_joint_index_by_name(self, name) -> int:
        """Get joint index by name"""
        for idx, joint in enumerate(self.joints):
            if joint.name == name:
                return idx
        raise ValueError("Joint not found")

    def reset_position_state(self):
        """Reset joint positions to default"""
        self.position_state = np.tile(
            [joint.default() for joint in self.joints],
            (self.position_state.shape[0], 1),
        )

    def get_position_state(self, name, dim=0) -> float:
        """Get joint position by name"""
        idx = self._get_joint_index_by_name(name)
        return self.position_state[dim][idx]

    def set_position_state(self, name, value, dim=0):
        """Set joint position by name"""
        idx = self._get_joint_index_by_name(name)
        if self.joints[idx].type == "fixed":
            raise ValueError(f"Joint {name} is fixed")
        if not self.joints[idx].is_within_bounds(value):
            raise ValueError(f"Value {value} is out of bounds for joint {name}")
        self.position_state[dim][idx] = value

    def get_position_state_actual(self):
        return self.position_state[0]

    def get_position_state_actual_with_mask(self):
        col_offset = []
        for idx, mask in enumerate(self.joint_mask):
            if mask == 1:
                col_offset.append(idx)

        return self.position_state[:, col_offset]

    def get_position_state_projected(self):
        return self.position_state[1]

    # TODO: Return the first demension
    def get_position_error(self):
        return np.diff(self.position_state, axis=0)

    # TODO: Maybe remove?
    def get_position_state_full(self):
        return np.vstack((self.position_state, self.get_position_error()))

    # TODO: Maybe remove?
    def is_objective_reached(self, tolerance=0.005):
        return np.allclose(
            self.get_position_error()[0][1:4], 0.0, atol=tolerance
        )  # TODO: Remove 0:1:4

    # def forward_orientation(self):
    #     joint_rotation = np.array([0, 0, 0])
    #     frame = np.empty((0, 3))
    #     for joint, joint_parameter in zip(
    #         self.joints, self.get_position_state_actual()
    #     ):
    #         joint_joint = np.array([0, 0, 0])
    #         if joint.origin_orientation is not None:
    #             joint_joint = joint.origin_orientation
    #         if joint.rotation is not None:
    #             rotation = joint.rotation * joint_parameter
    #             joint_joint = joint_joint + rotation
    #         joint_rotation = joint_rotation + joint_joint
    #         frame = np.append(frame, np.array([joint_rotation]), axis=0)

    #     return frame

    def _calculate_forward_kinematics(self, joint_parameters, joint_name=None):
        if len(self.joints) != len(joint_parameters):
            raise ValueError(
                f"joint_parameters has length {len(joint_parameters)} but robot has {len(self.joints)} joints"
            )

        euler_matrix = np.identity(4)
        rotation_matrix = np.array([0, 0, 0])
        frame_cumulative = np.empty((0, 6))
        for joint, joint_parameter in zip(self.joints, joint_parameters):
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
            frame_cumulative = np.append(
                frame_cumulative, np.array([coord_rot]), axis=0
            )

            if joint_name is not None and joint.name == joint_name:
                break

        return frame_cumulative

    def forward_kinematics2(self, joint_name=None) -> np.ndarray:
        frame = self._calculate_forward_kinematics(
            self.get_position_state_actual(), joint_name
        )

        if joint_name is not None:
            return frame[-1]

        return frame

    # TODO: Return 1x6 matrix
    def forward_kinematics(self, joint_name=None) -> np.ndarray:
        frame = self._forward_kinematics_frame(
            self.get_position_state_actual(), joint_name
        )
        return frame[:3, 3]

    def _forward_kinematics_frame(
        self, joint_parameters, joint_name=None
    ) -> np.ndarray:
        if len(self.joints) != len(joint_parameters):
            raise ValueError(
                f"joint_parameters has length {len(joint_parameters)} but robot has {len(self.joints)} joints"
            )

        frame_matrix = np.identity(4)
        for joint, joint_parameter in zip(self.joints, joint_parameters):
            if joint.origin_translation is not None:
                frame_matrix = frame_matrix.dot(
                    geom.Geometry.translation_matrix(*joint.origin_translation)
                )
            if joint.origin_orientation is not None:
                frame_matrix = frame_matrix.dot(
                    geom.Geometry.rotation_matrix(*joint.origin_orientation[1:])
                )
            if joint.rotation is not None:
                rotation = joint.rotation * joint_parameter
                frame_matrix = frame_matrix.dot(
                    geom.Geometry.rotation_matrix(*rotation[1:])
                )
            if joint.translation is not None:
                translation = joint.translation * joint_parameter
                frame_matrix = frame_matrix.dot(
                    geom.Geometry.translation_matrix(*translation)
                )

            if joint_name is not None and joint.name == joint_name:
                break

        return frame_matrix

    def inverse_kinematics(self, target_position) -> np.ndarray:
        if target_position.size != 3:
            raise ValueError(f"point has size {target_position.size} but should be 3")

        frame_matrix = np.identity(4)
        frame_matrix[:3, 3] = target_position

        target = frame_matrix[:3, 3]

        # TODO: The lower bound must also include the origin of the joint
        lb = [joint.lower_bound for joint in self.joints]
        ub = [joint.upper_bound for joint in self.joints]

        def optimize_function(x):
            # frame = self._forward_kinematics_frame(x, joint_name="attachment_joint")
            # fk = frame[:3, 3]

            frame = self._calculate_forward_kinematics(x, joint_name="attachment_joint")
            fk = frame[-1][:3]

            target_error = fk - target

            return target_error

        res = optimize.least_squares(
            optimize_function, self.get_position_state_actual(), bounds=(lb, ub)
        )
        if not res.success:
            raise ValueError("Could not find inverse kinematics solution")

        self.position_state[1] = res.x

        return res.x

    # TODO: Clean up
    def __str__(self) -> str:
        position_state = self.get_position_state_full().T

        s = f"Robot={self.name}"
        if self.model is not None:
            s += f", Model={self.model}"
        s += f", Links={len(self.links)}, Joints={len(self.joints)}"
        for joint in self.joints:
            s += f"\n  {joint}"
        s += "\n"
        s += f"EndEffectorVector={util.numpy3d_to_string(self.forward_kinematics2(joint_name='attachment_joint')[:3])}, EndEffectorOrientation={util.numpy3d_to_string(self.forward_kinematics2(joint_name='attachment_joint')[3:])}\n"
        s += f"ObjectiveReached={self.is_objective_reached()}\n"
        for idx, joint in enumerate(self.joints):
            s += f"  Joint={joint.name}, Actual={np.rad2deg(position_state[idx][0]):.2f}°, Projected={np.rad2deg(position_state[idx][1]):.2f}°, Error={np.rad2deg(position_state[idx][2]):.2f}°\n"
        return s

    # TODO: Move to util
    def plot_robot(self):
        import matplotlib
        import matplotlib.pyplot as plt
        import networkx as nx

        matplotlib.use("TkAgg")

        nx.draw_networkx(self.body)

        plt.show()
