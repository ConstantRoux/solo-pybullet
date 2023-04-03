import numpy as np
from solo_pybullet.math.matrix_math import R as Rot
from solo_pybullet.math.matrix_math import Th


class ParallelController:
    @staticmethod
    def controller(k, tx, ty, tz, rx, ry, rz, constraints):
        """
        Process the inputs to move the robot in the disired configuration
        :param k: robot wrapper
        :param tx: translation on x axis
        :param ty: translation on y axis
        :param tz: translation on z axis
        :param rx: rotation on x axis
        :param ry: rotation on y axis
        :param rz: rotation on z axis
        :param constraints: constrain the robot joints
        :return: desired configuration
        """
        # set T01 (foot to shoulder) using inputs
        R = Rot(rx, ry, rz)
        P = np.array([tx, ty, tz])
        T01 = Th(R, P)

        return k.body_inverse_kinematics(T01, constraints)
