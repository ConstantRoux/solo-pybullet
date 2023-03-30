import numpy as np
from solo_pybullet.math.matrix_math import R as Rot
from solo_pybullet.math.matrix_math import Th


class ParallelController:
    @staticmethod
    def controller(k, tx, ty, tz, rx, ry, rz, constraints):
        # set T01 (foot to shoulder) using inputs
        R = Rot(rx, ry, rz)
        P = np.array([tx, ty, tz])
        T01 = Th(R, P)

        return k.body_inverse_kinematics(T01, constraints)
