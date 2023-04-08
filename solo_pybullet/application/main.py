#####################
#  LOADING MODULES ##
#####################
import threading
from enum import Enum

import numpy as np
import pybullet as p
import time

from solo_pybullet.controller.hybrid_controller.HybridController import HybridController
from solo_pybullet.controller.initializer_controller.InitializerController import InitializerController
from solo_pybullet.interface.FootHolder import FootHolder
from solo_pybullet.interface.Gamepad import gamepad_thread
from solo_pybullet.interface.GamePadToRobot import wait_awakening_input, staticInput, walkInput
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.controller.parallel_controller.ParallelController import ParallelController


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
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)  # constraints for each joint
    safe_mode_duration = 1  # wanted time to move from pybullet init config to safe position (lying on the ground)
    idle_mode_duration = 1  # wanted time to move from safe position to idle position
    T = 0.5  # time period of one foot cycle during walk mode
    max_step_length = 0.2  # max step length doable by the robot
    H = 0.05  # height of the step

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
    k.kinematics.debug = False

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

    #################################
    #            REMOTE             #
    #################################
    threading.Thread(target=gamepad_thread, args=()).start()

    previousValuesStatic = np.zeros((6,))
    previousValuesStatic[2] = 0.16
    previousValuesWalk = np.zeros((3,))

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
            flag, Q, dQ = InitializerController.safe_configuration(k, dt * i, safe_mode_duration, constraints)
            if flag:
                state = 1

        # wait for awakening remote input
        elif state == 1:
            if wait_awakening_input():
                idle_start_time = dt * i
                state = 2

        # move robot to idle position
        elif state == 2:
            flag, Q, dQ = InitializerController.idle_configuration(k, dt * i - idle_start_time, idle_mode_duration, constraints)
            if flag:
                state = 2999

        elif state == 99:
            valuesStatic = staticInput(previousValuesStatic)

            Q = ParallelController.controller(k, *valuesStatic, constraints)
            previousValuesStatic = valuesStatic.copy()

        elif state == 2999:
            valuesWalk = walkInput(1)
            fh = FootHolder(k, np.array([-L[1] - L[2] - L[3] - L[5], -L[0]]), max_step_length, T)
            Pi, Pf = fh.get_feet_pos(*valuesWalk, H)
            P = np.zeros((12,))
            dP = np.zeros((12,))
            P[0:3] = CycloidFootTrajectory.f(dt * i + T / 2, T, Pi, Pf, dir=True)
            P[3:6] = CycloidFootTrajectory.f(dt * i, T, Pi, Pf, dir=True)
            P[6:9] = CycloidFootTrajectory.f(dt * i + T / 2, T, Pi, Pf, dir=False)
            P[9:12] = CycloidFootTrajectory.f(dt * i, T, Pi, Pf, dir=False)

            dP[0:3] = CycloidFootTrajectory.df(dt * i + T / 2, T, Pi, Pf, dir=True)
            dP[3:6] = CycloidFootTrajectory.df(dt * i, T, Pi, Pf, dir=True)
            dP[6:9] = CycloidFootTrajectory.df(dt * i + T / 2, T, Pi, Pf, dir=False)
            dP[9:12] = CycloidFootTrajectory.df(dt * i, T, Pi, Pf, dir=False)

            Q, dQ = HybridController.controller(k, *previousValuesStatic, P, dP, constraints)

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
