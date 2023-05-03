import numpy as np


class Geometry:
    @staticmethod
    def _ry_matrix(theta):
        """Rotation matrix around the Y axis"""
        return np.array([
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])

    @staticmethod
    def _rz_matrix(theta):
        """Rotation matrix around the Z axis"""
        return np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
        ])

    @staticmethod
    def translation_matrix(x, y, z):
        return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

    @staticmethod
    def rotation_matrix(pitch, yaw):
        cartesian_matrix = np.dot(Geometry._rz_matrix(yaw), Geometry._ry_matrix(pitch))

        homogeneous_matrix = np.identity(4)
        homogeneous_matrix[:-1, :-1] = cartesian_matrix

        return homogeneous_matrix
