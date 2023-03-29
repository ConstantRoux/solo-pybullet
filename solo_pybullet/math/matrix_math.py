import numpy as np
from numpy import cos as c
from numpy import sin as s


def Th(R, P):
    R = np.append(R, [[0, 0, 0]], 0)
    P = np.append(P, [1], 0)
    return np.column_stack([R, P])


def Rx(rx):
    return np.matrix([[1, 0, 0],
                      [0, c(rx), -s(rx)],
                      [0, s(rx), c(rx)]])


def Ry(ry):
    return np.matrix([[c(ry), 0, s(ry)],
                      [0, 1, 0],
                      [-s(ry), 0, c(ry)]])


def Rz(rz):
    return np.matrix([[c(rz), -s(rz), 0],
                      [s(rz), c(rz), 0],
                      [0, 0, 1]])


def R(rx, ry, rz):
    return Rx(rx) @ Ry(ry) @ Rz(rz)

