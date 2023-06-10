import numpy as np


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
