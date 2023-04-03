#####################
#  LOADING MODULES ##
#####################
import numpy as np
from solo_pybullet.controller.kinematic_controller.params import *


def foot_trajectory(t, x0, y0, z0):
    """
    First atempte of foot trajectory (OLD)
    :param t: Current time in the simulation in second
    :param x0: initial position on x axis
    :param y0: initial position on y axis
    :param z0: initial position on z axis
    :return: Feet position in space
    """
    T, xF0, xH0, yF0, yH0, z0, dx, dz = update_params()
    x = []
    y = []
    z = []
    if t >= T:
        t %= T
    x.append(x0 - dx * np.cos(2 * np.pi * t / T))
    if t <= T / 2.:
        z.append(z0 + dz * np.sin(2 * np.pi * t / T))
    else:
        z.append(z0)
    y.append(y0)
    return np.matrix([x, y, z, [1]])


def d_foot_trajectory(t):  # arguments : time
    """

    :param t:Current time in the simulation in second
    :return: Feet velocity in space
    """
    x = []
    y = []
    z = []
    if t >= T:
        t %= T
    x.append((2 * np.pi * dx / T) * np.sin(2 * np.pi * t / T))
    if t <= T / 2.:
        z.append((2 * np.pi * dz / T) * np.cos(2 * np.pi * t / T))
    else:
        z.append(0)
    y.append(0)
    return np.matrix([x, y, z, [1]])
