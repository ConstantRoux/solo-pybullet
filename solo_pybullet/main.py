#####################
#  LOADING MODULES ##
#####################
import pybullet as p
from solo_pybullet.initialization_simulation import configure_simulation
from solo_pybullet.controller.kinematic_controller.kinematic_controller import kinematic_controller

if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    dt = 0.001  # define the time step in second
    duration = 3600  # define the duration of the simulation in seconds
    robot_id, robot_wrapper, rev_joint_idx = configure_simulation(dt)  # configure and load model in pybullet and pinocchio


    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # compute desired configuration
        q, dq = kinematic_controller(robot_id, rev_joint_idx, dt * i)

        # active actuators with new configuration
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL, targetPositions=q, targetVelocities=dq)

        # next step simulation
        p.stepSimulation()

    # quit pybullet
    p.disconnect()
