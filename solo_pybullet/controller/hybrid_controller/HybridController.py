import numpy as np
from solo_pybullet.math.matrix_math import R as Rot
from solo_pybullet.math.matrix_math import Th


class HybridController:
    @staticmethod
    def controller(k, tx, ty, tz, rx, ry, rz, P, dP, constraints):
        """
        Process the inputs to move the robot in the disired configuration
        :param k: robot wrapper
        :param tx: translation on x axis of the base
        :param ty: translation on y axis of the base
        :param tz: translation on z axis of the base
        :param rx: rotation on x axis of the base
        :param ry: rotation on y axis of the base
        :param rz: rotation on z axis of the base
        :param P: local positions of each foot
        :param dP: local velocities of each foot
        :param constraints: constrain the robot joints
        :return: desired configuration
        """
        # set T01 (foot to shoulder) using inputs
        T01 = Th(Rot(rx, ry, rz), np.array([tx, ty, tz]))

        return k.body_inverse_kinematics(T01, P, dP, constraints)
