#####################
#  LOADING MODULES ##
#####################
import threading
import numpy as np
import pybullet as p
import time
from solo_pybullet.controller.hybrid_controller.HybridController import HybridController
from solo_pybullet.controller.hybrid_controller.LoggerThread import logger_thread, expected_speed_x, expected_speed_y, reached_speed_x, reached_speed_y
from solo_pybullet.controller.hybrid_controller.RobotMode import RobotMode
from solo_pybullet.controller.initializer_controller.InitializerController import InitializerController
from solo_pybullet.controller.hybrid_controller.FootHolder import FootHolder
from solo_pybullet.interface.Gamepad import gamepad_thread
from solo_pybullet.interface.Gamepad2Controller import wait_awakening_input, get_static_input, get_walk_input, \
    get_new_mode
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper
from solo_pybullet.simulation.initialization_simulation import configure_simulation


def main():
    #################################
    #          PARAMETERS           #
    #################################
    # robot parameters
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]  # length between each joint for the four legs of the robot
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)  # constraints for each joint
    safe_mode_duration = 1  # wanted time to move from pybullet init config to safe position (lying on the ground)
    idle_mode_duration = 1  # wanted time to move from safe position to idle position
    T = 0.3  # time period of one foot cycle during walk mode
    max_step_length = 0.3  # max step length doable by the robot
    H = 0.015  # height of the step
    Vmax = 0.5  # max speed reachable by the base of the robot

    # simulation parameters
    dt = 0.01  # define the time step in second
    sim_speed = 1  # to slow down by sim_speed the simulation
    sim_duration = 90  # duration of the simulation in seconds

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
    # variables for gamepad control
    threading.Thread(target=gamepad_thread, args=()).start()
    previous_values_static = np.zeros((6,))
    previous_values_static[2] = 0.16
    values_walk = np.zeros((3,))

    #################################
    #           DEBUGGER            #
    #################################
    debug = True
    if debug:
        threading.Thread(target=logger_thread, args=(dt, Vmax)).start()

    #################################
    #            LOGGER             #
    #################################
    logger = False
    data_log = None
    if logger:
        data_log = np.zeros((int(sim_duration / dt), 1+2+24))

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
                mode = RobotMode(1)
                state = 3

        # check control mode
        if state == 3:
            new_mode = get_new_mode(mode)
            # if a new mode is wanted
            if new_mode != mode:
                mode = new_mode
                state = 4
            else:
                state = 5

        # init robot fot the next mode
        if state == 4:
            if mode == RobotMode.SAFETY_MODE:
                state = 41
            elif mode == RobotMode.STATIC_MODE:
                state = 42
            elif mode == RobotMode.WALK_MODE:
                state = 43

        # init robot to SAFETY_MODE
        if state == 41:
            state = 3

        # init robot to STATIC_MODE
        elif state == 42:
            state = 3

        # init robot to WALK_MODE
        elif state == 43:
            state = 3

        # execute the desired mode
        if state == 5:
            if mode == RobotMode.SAFETY_MODE:
                state = 51
            elif mode == RobotMode.STATIC_MODE:
                state = 52
            elif mode == RobotMode.WALK_MODE:
                state = 53

        # execute the mode SAFETY_MODE
        if state == 51:
            state = 3

        # execute the mode STATIC_MODE
        elif state == 52:
            valuesStatic = get_static_input(previous_values_static)
            Q, dQ = HybridController.controller(k, *valuesStatic, np.array([-L[1] - L[2] - L[3] - L[5], -L[0], 0] * 4), np.zeros((12,)), constraints)
            previous_values_static = valuesStatic.copy()
            state = 3

        # execute the mode WALK_MODE
        elif state == 53:
            values_walk = get_walk_input(Vmax)

            if debug:
                expected_speed_x.append(-values_walk[0])
                expected_speed_y.append(values_walk[1])
                lv, _ = p.getBaseVelocity(robot_id)
                reached_speed_x.append(lv[0])
                reached_speed_y.append(lv[1])

            fh = FootHolder(k, np.array([-L[1] - L[2] - L[3] - L[5], -L[0]]), max_step_length, T)
            Pi, Pf = fh.get_feet_pos(*values_walk, H)
            P = np.zeros((12,))
            dP = np.zeros((12,))
            P[0:3] = CycloidFootTrajectory.f(dt * i + T / 2, T, Pi[0:3], Pf[0:3], dir=True)
            P[3:6] = CycloidFootTrajectory.f(dt * i, T, Pi[3:6], Pf[3:6], dir=True)
            P[6:9] = CycloidFootTrajectory.f(dt * i, T, Pi[6:9], Pf[6:9], dir=True)
            P[9:12] = CycloidFootTrajectory.f(dt * i + T / 2, T, Pi[9:12], Pf[9:12], dir=True)

            dP[0:3] = CycloidFootTrajectory.df(dt * i + T / 2, T, Pi[0:3], Pf[0:3], dir=True)
            dP[3:6] = CycloidFootTrajectory.df(dt * i, T, Pi[3:6], Pf[3:6], dir=True)
            dP[6:9] = CycloidFootTrajectory.df(dt * i, T, Pi[6:9], Pf[6:9], dir=True)
            dP[9:12] = CycloidFootTrajectory.df(dt * i + T / 2, T, Pi[9:12], Pf[9:12], dir=True)

            Q, dQ = HybridController.controller(k, *previous_values_static, P, dP, constraints)
            state = 3

        #################################
        #            LOGGER             #
        #################################
        if logger:
            data_log[i, :] = np.concatenate((np.array([dt]), values_walk[0:2], Q, dQ))

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

            #################################
            #          SIMULATION           #
            #################################
            # uncomment to have autofocus camera on the robot
            basePos, baseOrn = p.getBasePositionAndOrientation(robot_id)  # Get model position
            eulerBasOrn = p.getEulerFromQuaternion(baseOrn)
            p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=eulerBasOrn[2] * 180 / np.pi - 90, cameraPitch=-30,
                                         cameraTargetPosition=basePos)

    # quit pybullet
    p.disconnect()

    #################################
    #            LOGGER             #
    #################################
    if logger:
        np.savetxt("data" + str(dt) + ".csv", data_log, delimiter=",")


if __name__ == '__main__':
    main()
