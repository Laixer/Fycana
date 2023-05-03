import numpy as np


def shortest_rotation(distance):
    """ Calculate the shortest distance between two points on a circle """

    dist_normal = (distance + (2.0 * np.pi)) % (2.0 * np.pi)

    if dist_normal > np.pi:
        return dist_normal - (2.0 * np.pi)

    return dist_normal
