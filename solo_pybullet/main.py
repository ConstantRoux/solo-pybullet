#####################
#  LOADING MODULES ##
#####################
import time

import numpy as np
import pybullet as p
import pinocchio as pin
from solo_pybullet.controller2 import c_walking_IK
from solo_pybullet.initialization_simulation import configure_simulation, get_pos_vel_joints


if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    dt = 0.001  # define the time step in second
    duration = 3600  # define the duration of the simulation in seconds
    robot_id, robot_wrapper, rev_joint_idx = configure_simulation(dt)  # configure and load model in pybullet and
    # pinocchio

    # for i in range(p.getNumJoints(robot_id)):
    #     print(p.getJointInfo(robot_id, i))

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # get pos and vel of all joints
        q, dq = get_pos_vel_joints(robot_id, rev_joint_idx)

        # call controler to get all joint torques -- TODO
        joint_torques = c_walking_IK(q, dq, dt, robot_wrapper, dt * i)

        # set control torques for all joints in PyBullet
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.TORQUE_CONTROL, forces=joint_torques)

        # next step simulation
        p.stepSimulation()

        # time.sleep(0.1)

    # quit pybullet
    p.disconnect()
