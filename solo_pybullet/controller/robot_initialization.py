import numpy as np
import pybullet as p

from solo_pybullet.simulation.initialization_simulation import configure_simulation


def safeconfiguration(robot_id, rev_joint_idx):
    """
    Initialize the robot by moving is joints in the safe position
    :param robot_id:
    :param rev_joint_idx:
    :return: ?? (Failure?, elapsed time?)
    """
    pi = np.pi
    direction = np.array([1, 1, -1, 1, 1, 1, 1, -1, -1, 1, -1, 1])
    q = np.array([0, pi/2, pi, 0, pi/2, - pi, 0, pi/2, - pi, 0, pi/2, pi]) * direction
    dq = np.ones((12,)) * 0.0001 * direction

    p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                targetPositions=q, targetVelocities=dq)

def idleconfiguration(robot_id, rev_joint_idx):
    """

    :param robot_id:
    :param rev_joint_idx:
    :return:
    """
    pi = np.pi
    direction = np.array([1, 1, -1, 1, 1, -1, 1, -1, 1, 1, -1, 1])
    q = np.array([0, pi/4, pi/2, 0, pi/4, pi/2, 0, pi/4, pi/2, 0, pi/4, pi/2]) * direction

    dq = np.ones((12,)) * 0.001 * direction

    p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                targetPositions=q, targetVelocities=dq)