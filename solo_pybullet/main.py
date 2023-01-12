#####################
#  LOADING MODULES ##
#####################
import pybullet as p
import pinocchio as pin
from solo_pybullet.controller import c_walking_ID
from solo_pybullet.initialization_simulation import configure_simulation, get_pos_vel_joints


if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    dt = 0.001  # define the time step in second
    duration = 10  # define the duration of the simulation in seconds
    robot_id, model, rev_joint_idx = configure_simulation(dt)  # configure and load model in pybullet and pinocchio

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # get pos and vel of all joints
        q, dq = get_pos_vel_joints(robot_id, rev_joint_idx)

        # call controler to get all joint torques -- TODO
        # joint_torques = c_walking_ID(q, dq, dt, model, dt * i)

        # set control torques for all joints in PyBullet
        # p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.TORQUE_CONTROL, forces=joint_torques)

        # TEST


        # next step simulation
        p.stepSimulation()

    # quit pybullet
    p.disconnect()
