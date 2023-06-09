import os
import numpy as np


# Obsolete
def numpy3d_to_string(array):
    """Convert a numpy array to a string"""
    if array is None:
        return "None"
    return f"[{array[0]:.2f}, {array[1]:.2f}, {array[2]:.2f}]"


def format_angle(value):
    return "{:.2f}°".format(np.rad2deg(value))


def format_angle_both(value):
    return "{:.3f}  {:.2f}°".format(value, np.rad2deg(value))


def format_coord(value):
    return "{:.2f}".format(value)


def format_euler_tuple(effector):
    return "({}, {}, {}) [{}, {}, {}]".format(
        format_coord(effector[0]),
        format_coord(effector[1]),
        format_coord(effector[2]),
        format_angle(effector[3]),
        format_angle(effector[4]),
        format_angle(effector[5]),
    )
