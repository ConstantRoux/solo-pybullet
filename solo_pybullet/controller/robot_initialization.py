import numpy as np
import pybullet as p

from solo_pybullet.controller.parallel_controller.ParallelController import ParallelController
from solo_pybullet.math.matrix_math import R as Rot
from solo_pybullet.math.matrix_math import Th


def safe_configuration(k, t, duration):
    """
    Initialize the robot by moving is joints so the robot lying on the ground
    :param k: bullet wrapper
    :param t: current time of the simulation
    :param duration: time it take to go from current configuration to the safe configuration
    :return: True if the movement is finish, else False, joint configuration
    """
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)  # set the joints limits

    tz = (- 0.32 / duration) * t + 0.32  # compute the target height

    # set T01 (foot to shoulder) using inputs
    R = Rot(0, 0, 0)  # no rotation during the movement
    P = np.array([0, 0, tz])  # move only on z
    T01 = Th(R, P)  # compute the homogeneous matrix

    Q = k.body_inverse_kinematics(T01, constraints)  # compute the configuration

    return (t >= duration), Q


def idle_configuration(k, t, duration, h=0.18):
    """
    Move the robot's joints so the robot stand in idle position
    :param k: bullet wrapper
    :param t: current time of the simulation
    :param duration: time it take to go from current configuration to the idle configuration
    :param h: target height of the robot
    :return: If the movement is ended, joints configuration
    """

    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)  # set the joints limits

    tz = (h / duration) * t  # compute the desired height

    # set T01 (foot to shoulder) using inputs
    R = Rot(0, 0, 0)  # no rotation during the movement
    P = np.array([0, 0, tz])  # move only on z
    T01 = Th(R, P)  # compute the homogeneous matrix

    Q = k.body_inverse_kinematics(T01, constraints)  # compute the configuration

    return (t > duration), Q
