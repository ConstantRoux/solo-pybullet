import numpy as np
from numpy import cos as c
from numpy import sin as s

from solo_pybullet.math.matrix_math import Th


class ParallelController:
    @staticmethod
    def R(rx, ry, rz):
        Rx = np.matrix([[1, 0, 0],
                        [0, c(rx), -s(rx)],
                        [0, s(rx), c(rx)]])
        Ry = np.matrix([[c(ry), 0, s(ry)],
                        [0, 1, 0],
                        [-s(ry), 0, c(ry)]])
        Rz = np.matrix([[c(rz), -s(rz), 0],
                        [s(rz), c(rz), 0],
                        [0, 0, 1]])
        return Rx @ Ry @ Rz

    @staticmethod
    def controller(k, tx, ty, tz, rx, ry, rz):
        constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)

        # set T01 (foot to shoulder) using inputs
        R = ParallelController.R(rx, ry, rz)
        P = np.array([tx, ty, tz])
        T01 = Th(R, P)

        return k.body_inverse_kinematics(T01, constraints)
