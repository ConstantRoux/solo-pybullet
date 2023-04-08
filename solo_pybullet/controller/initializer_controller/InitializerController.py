import numpy as np
from solo_pybullet.math.matrix_math import R as Rot
from solo_pybullet.math.matrix_math import Th


class InitializerController:
    @staticmethod
    def safe_configuration(k, t, duration, constraints):
        """
        Initialize the robot by moving is joints so the robot lying on the ground
        :param k: bullet wrapper
        :param t: current time of the simulation
        :param duration: time it take to go from current configuration to the safe configuration
        :return: True if the movement is finish, else False, joint configuration
        """
        tz = (- 0.32 / duration) * t + 0.32  # compute the target height

        # set T01 (foot to shoulder) using inputs
        R = Rot(0, 0, 0)  # no rotation during the movement
        P = np.array([0, 0, tz])  # move only on z
        T01 = Th(R, P)  # compute the homogeneous matrix

        Q, dQ = k.body_inverse_kinematics(T01, np.array([-k.L[1] - k.L[2] - k.L[3] - k.L[5], -k.L[0], 0] * 4), np.zeros((12,)), constraints)  # compute the configuration

        return (t >= duration), Q, dQ

    @staticmethod
    def idle_configuration(k, t, duration, constraints, h=0.16):
        """
        Move the robot's joints so the robot stand in idle position
        :param constraints: set the joints limits
        :param k: bullet wrapper
        :param t: current time of the simulation
        :param duration: time it take to go from current configuration to the idle configuration
        :param h: target height of the robot
        :return: If the movement is ended, joints configuration
        """
        tz = (h / duration) * t  # compute the desired height

        # set T01 (foot to shoulder) using inputs
        R = Rot(0, 0, 0)  # no rotation during the movement
        P = np.array([0, 0, tz])  # move only on z
        T01 = Th(R, P)  # compute the homogeneous matrix

        Q, dQ = k.body_inverse_kinematics(T01, np.array([-k.L[1] - k.L[2] - k.L[3] - k.L[5], -k.L[0], 0] * 4), np.zeros((12,)), constraints)  # compute the configuration

        return (t > duration), Q, dQ
