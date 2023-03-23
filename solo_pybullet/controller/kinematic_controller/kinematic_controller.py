#####################
#  LOADING MODULES ##
#####################
import pybullet as p
import numpy as np
# from solo_pybullet.math.matrix_math import transformation_matrix
from solo_pybullet.controller.kinematic_controller.foot_trajectory import foot_trajectory, d_foot_trajectory
from solo_pybullet.controller.kinematic_controller.params import *


def get_transformation_matrix(robot_id):
    T, Q = p.getBasePositionAndOrientation(robot_id)
    R = np.reshape(p.getMatrixFromQuaternion(Q), (3, 3))
    return transformation_matrix(R, np.array(T))


def get_jacobian(robot_id, feet_idx, q_pos):
    J = []
    for i in range(len(feet_idx)):
        com_trn = p.getLinkState(robot_id, feet_idx[i])[2]
        J_t, _ = p.calculateJacobian(robot_id, feet_idx[i], com_trn, q_pos, [0.] * len(q_pos), [0.] * len(q_pos))
        J.append(np.array(J_t)[:, 6+3*i:6+3*(i+1)])
    return J


def get_dq(J, vel):
    dq = np.zeros((12, 1))

    for i, v in enumerate(vel):
        dq[3*i:3*(i+1)] = np.linalg.pinv(J[i]) @ v

    return dq


def kinematic_controller(robot_id, rev_joint_idx, t):
    # parameters
    end_effectors = [3, 7, 11, 15]  # TODO function : get foots id
    T, xF0, xH0, yF0, yH0, z0, dx, dz = update_params()

    # get the transformation matrix to convert local coord to world coord
    loc2world = get_transformation_matrix(robot_id)

    # get desired position of each foot
    p1 = (loc2world @ foot_trajectory(t, xF0, yF0, z0))[0:3, 0]
    p2 = (loc2world @ foot_trajectory(t + T / 2, xF0, yH0, z0))[0:3, 0]
    p3 = (loc2world @ foot_trajectory(t + T / 2, xH0, yF0, z0))[0:3, 0]
    p4 = (loc2world @ foot_trajectory(t, xH0, yH0, z0))[0:3, 0]
    pos = [p1, p2, p3, p4]

    # get desired velocities of each foot
    v1 = (d_foot_trajectory(t))[0:3, 0]
    v2 = (d_foot_trajectory(t + T / 2))[0:3, 0]
    v3 = (d_foot_trajectory(t + T / 2))[0:3, 0]
    v4 = (d_foot_trajectory(t))[0:3, 0]
    vel = [v1, v2, v3, v4]

    # compute desired q of each actuator using inverse kinematic model
    q = p.calculateInverseKinematics2(robot_id, end_effectors, pos)

    # compute desired dq of each actuator using jacobian
    J = get_jacobian(robot_id, end_effectors, q)
    dq = get_dq(J, vel)

    return q, dq
