import numpy as np
from numpy import cos as c
from numpy import sin as s


def Th(R, P):
    R = np.append(R, [[0, 0, 0]], 0)
    P = np.append(P, [1], 0)
    return np.column_stack([R, P])


def Rx(t):
    return np.matrix([[1, 0, 0],
                      [0, c(t), -s(t)],
                      [0, s(t), c(t)]])


def Ry(t):
    return np.matrix([[c(t), 0, s(t)],
                      [0, 1, 0],
                      [-s(t), 0, c(t)]])


def Rz(t):
    return np.matrix([[c(t), -s(t), 0],
                      [s(t), c(t), 0],
                      [0, 0, 1]])
