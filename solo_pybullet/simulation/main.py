#####################
#  LOADING MODULES ##
#####################
import pybullet as p
import time

from solo_pybullet.controller.robot_initialization import safe_configuration
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.controller.kinematic_controller.kinematic_controller import kinematic_controller


def main():
    #################################
    #          PARAMETERS           #
    #################################
    # robot parameters
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]  # length between each joint for the four legs of the robot

    # simulation parameters
    dt = 0.01  # define the time step in second
    sim_speed = 1  # to slow down by sim_speed the simulation
    sim_duration = 600  # duration of the simulation in seconds

    #################################
    #       INITIALIZATION          #
    #################################
    # init simulation
    robot_id, rev_joint_idx = configure_simulation(dt, True)

    # init robot model
    k = BulletWrapper(L)
    k.kinematics.debug = True

    # init state machine
    state = 0

    #################################
    #          SIMULATION           #
    #################################
    for i in range(int(sim_duration / dt)):
        #################################
        #           REAL TIME           #
        #################################
        t0 = time.perf_counter()

        #################################
        #         STATE MACHINE         #
        #################################
        # init robot
        if state == 0:
            flag, Q = safe_configuration()

        #################################
        #        PD-CONTROL LOOP        #
        #################################
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=Q)  # targetVelocities=dq

        #################################
        #           REAL TIME           #
        #################################
        # next step simulation
        p.stepSimulation()

        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep * sim_speed)  # to slow down by sim_speed the simulation

    # quit pybullet
    p.disconnect()


if __name__ == '__main__':
    main()
