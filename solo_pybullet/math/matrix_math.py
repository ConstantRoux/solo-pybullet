import numpy as np
from numpy import cos as c
from numpy import sin as s


def Th(R, P):
    """
    Compute the homogeneous matrix
    :param R: Rotation matrix
    :param P: Vector for translation
    :return: Homogeneous matrix
    """
    R = np.append(R, [[0, 0, 0]], 0)
    P = np.append(P, [1], 0)
    return np.column_stack([R, P])


def Rx(rx):
    """
    Rotation matrix around x axis
    :param rx: angle in radiant
    :return: Rotation matrix
    """
    return np.matrix([[1, 0, 0],
                      [0, c(rx), -s(rx)],
                      [0, s(rx), c(rx)]])


def Ry(ry):
    """
    Rotation matrix around y axis
    :param ry: angle in radiant
    :return: Rotation matrix
    """
    return np.matrix([[c(ry), 0, s(ry)],
                      [0, 1, 0],
                      [-s(ry), 0, c(ry)]])


def Rz(rz):
    """
    Rotation matrix around z axis
    :param rz: angle in radiant
    :return: Rotation matrix
    """
    return np.matrix([[c(rz), -s(rz), 0],
                      [s(rz), c(rz), 0],
                      [0, 0, 1]])


def R(rx, ry, rz):
    """
    Rotation matrix around x, y, z axis
    :param rx: angle around x axis in radiant
    :param ry: angle around y axis in radiant
    :param rz: angle around z axis in radiant
    :return: Rotation matrix
    """
    return Rx(rx) @ Ry(ry) @ Rz(rz)

