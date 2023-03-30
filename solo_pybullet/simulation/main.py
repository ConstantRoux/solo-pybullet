#####################
#  LOADING MODULES ##
#####################
from enum import Enum

import numpy as np
import pybullet as p
import time

from solo_pybullet.controller.robot_initialization import safe_configuration, idle_configuration
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.controller.kinematic_controller.kinematic_controller import kinematic_controller


class RobotMode(Enum):
    SAFETY_MODE = 0
    STATIC_MODE = 1
    WALK_MODE = 2

def main():
    #################################
    #          PARAMETERS           #
    #################################
    # robot parameters
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]  # length between each joint for the four legs of the robot
    safe_mode_duration = 1  # wanted time to move from pybullet init config to safe position (lying on the ground)
    idle_mode_duration = 1  # wanted time to move from safe position to idle position

    # simulation parameters
    dt = 0.01  # define the time step in second
    sim_speed = 1  # to slow down by sim_speed the simulation
    sim_duration = 600  # duration of the simulation in seconds

    #################################
    #       INITIALIZATION          #
    #################################
    # init simulation
    robot_id, rev_joint_idx = configure_simulation(dt, False)

    # init robot model
    k = BulletWrapper(L)
    k.kinematics.debug = True

    # init state machine
    state = 0
    mode = RobotMode(0)

    #################################
    #          SIMULATION           #
    #################################
    # local variables for simulation
    idle_start_time = None
    Q = np.zeros((12,))
    dQ = np.zeros((12,))

    for i in range(int(sim_duration / dt)):
        #################################
        #           REAL TIME           #
        #################################
        t0 = time.perf_counter()

        #################################
        #            REMOTE             #
        #################################
        # TODO : get remote data

        #################################
        #         STATE MACHINE         #
        #################################
        # init robot
        if state == 0:
            flag, Q = safe_configuration(k, dt * i, safe_mode_duration)
            if flag:
                state = 1

        # wait for awakening remote input
        elif state == 1:
            # TODO : get awakening input from the remote
            if True:
                idle_start_time = dt * i
                state = 2

        # move robot to idle position
        elif state == 2:
            flag, Q = idle_configuration(k, dt * i - idle_start_time, idle_mode_duration)
            if flag:
                state = 3

        # check control mode
        elif state == 3:
            # TODO : update control mode from remote input
            new_mode = RobotMode(0)

            # if a new mode is wanted
            if new_mode != mode:
                mode = new_mode
                state = 4

            else:
                state = 5

        # init robot fot the next mode
        elif state == 4:
            if mode == RobotMode.SAFETY_MODE:
                state = 41
            elif mode == RobotMode.STATIC_MODE:
                state = 42
            elif mode == RobotMode.WALK_MODE:
                state = 43

        # init robot to SAFETY_MODE
        elif state == 41:
            state = 3

        # init robot to STATIC_MODE
        elif state == 42:
            state = 3

        # init robot to WALK_MODE
        elif state == 43:
            state = 3

        # execute the desired mode
        elif state == 5:
            if mode == RobotMode.SAFETY_MODE:
                state = 51
            elif mode == RobotMode.STATIC_MODE:
                state = 52
            elif mode == RobotMode.WALK_MODE:
                state = 53

        # execute the mode SAFETY_MODE
        elif state == 51:
            state = 3

        # execute the mode STATIC_MODE
        elif state == 52:
            # TODO : convert remote inputs into commands (Tx, Ty, Tz, Rx, Ry, Rz)
            state = 3

        # execute the mode WALK_MODE
        elif state == 53:
            state = 3

        #################################
        #        PD-CONTROL LOOP        #
        #################################
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=Q, targetVelocities=dQ)

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
