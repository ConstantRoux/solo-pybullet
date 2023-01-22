#####################
#  LOADING MODULES ##
#####################
import pybullet as p
import numpy as np
from solo_pybullet.math.matrix_transformation import transformation_matrix
from solo_pybullet.controller.foot_trajectory import foot_trajectory


def get_transformation_matrix(robot_id):
    T, Q = p.getBasePositionAndOrientation(robot_id)
    R = np.reshape(p.getMatrixFromQuaternion(Q), (3, 3))
    return transformation_matrix(R, np.array(T))


def kinematic_controller(robot_id, t):
    # parameters
    T = 0.5  # period of the foot trajectory
    xF0 = 0.207  # initial X position of the front feet
    xH0 = -0.207  # initial X position of the hind feet
    yF0 = 0.14695  # initial Y position of the front feet
    yH0 = -0.14695  # initial Y position of the hind feet
    z0 = -0.25  # negative distance between the base and the ground
    end_effectors = [3, 7, 11, 15]

    # get the transformation matrix to convert local coord to world coord
    loc2world = get_transformation_matrix(robot_id)

    # get desired position of each foot
    p1 = (loc2world @ foot_trajectory(t, xF0, yF0, z0))[0:3, 0]
    p2 = (loc2world @ foot_trajectory(t + T / 2, xF0, yH0, z0))[0:3, 0]
    p3 = (loc2world @ foot_trajectory(t + T / 2, xH0, yF0, z0))[0:3, 0]
    p4 = (loc2world @ foot_trajectory(t, xH0, yH0, z0))[0:3, 0]

    # compute desired q of each actuator using inverse kinematic model
    q = p.calculateInverseKinematics2(robot_id, end_effectors, [p1, p2, p3, p4])

    return q
