#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p

from solo_pybullet.controller.OptimizationController import OptimizationController
from solo_pybullet.controller.parallel_controller.ParallelController import ParallelController
from solo_pybullet.controller.parallel_controller.Parameters import Parameters
from solo_pybullet.controller.robot_initialization import safeconfiguration, idleconfiguration
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.logger.Logger import Logger
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.foot_trajectory.BezierFootTrajectory import BezierFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper


def test_1():
    ####################
    #  INITIALIZATION ##
    ####################
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)
    k = BulletWrapper(L)

    T = 1
    Lpx = 0.
    Lpy = 0.15
    x0 = -L[1] - L[2] - L[3]
    y0 = -(L[0] + Lpy/2)
    z0 = -0.2
    H = 0.05

    duration = 1000 * T  # define the duration of the simulation in seconds
    dt = 0.01  # define the time step in second
    robot_id, rev_joint_idx = configure_simulation(dt, False)

    Pc = np.array([[0, 0], [2.5, 2.5], [5, 5]])
    controller = OptimizationController(k, T, dt, H, 0.2, z0, Pc)
    #      LOGGER     ##
    ####################
    log = True
    if log:
        l = Logger()
        titles = [r'expected $q$', r'simulated $q$', r'expected $\dot{q}$', r'simulated $\dot{q}$',
                  r'expected $P$', r'simulated $P$', r'expected $\dot{P}$', r'simulated $\dot{P}$']
        for i in range(8):
            l.create_channel(titles[i], np.zeros((12, int(duration / dt))))

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # init
        if dt * i < T:
            P = np.zeros((12,))

            P[0:3] = CycloidFootTrajectory.f(T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[3:6] = CycloidFootTrajectory.f(0, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[6:9] = CycloidFootTrajectory.f(T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            P[9:12] = CycloidFootTrajectory.f(0, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

            dP = np.zeros((12,))
            dP[0:3] = CycloidFootTrajectory.df(T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[3:6] = CycloidFootTrajectory.df(0, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[6:9] = CycloidFootTrajectory.df(T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            dP[9:12] = CycloidFootTrajectory.df(0, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

            # compute desired configuration
            q, dq = k.inverse_kinematics(P, dP, constraints)

            # logger
            if log:
                l.data[0][1][:, i] = q
                l.data[2][1][:, i] = dq
                l.data[4][1][:, i] = P
                l.data[6][1][:, i] = dP

                p_Q = p.getJointStates(robot_id, rev_joint_idx)
                sq = np.zeros((12,))
                sdq = np.zeros((12,))
                for j in range(12):
                    sq[j] = p_Q[j][0]
                    sdq[j] = p_Q[j][1]
                l.data[1][1][:, i] = sq
                l.data[3][1][:, i] = sdq
                l.data[5][1][:, i], l.data[7][1][:, i] = k.forward_kinematics(sq, sdq)

            p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                        targetPositions=q, targetVelocities=dq)
        else:
            P = np.zeros((12,))
            P[0:3] = CycloidFootTrajectory.f(dt * i + T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[3:6] = CycloidFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[6:9] = CycloidFootTrajectory.f(dt * i + T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            P[9:12] = CycloidFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

            dP = np.zeros((12,))
            dP[0:3] = CycloidFootTrajectory.df(dt * i + T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[3:6] = CycloidFootTrajectory.df(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[6:9] = CycloidFootTrajectory.df(dt * i + T/2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            dP[9:12] = CycloidFootTrajectory.df(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

            # compute desired configuration
            q, dq = k.inverse_kinematics(P, dP, constraints)

            # logger
            if log:
                l.data[0][1][:, i] = q
                l.data[2][1][:, i] = dq
                l.data[4][1][:, i] = P
                l.data[6][1][:, i] = dP

            p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                        targetPositions=q, targetVelocities=dq)

            # logger
            if log:
                p_Q = p.getJointStates(robot_id, rev_joint_idx)
                for j in range(12):
                    q[j] = p_Q[j][0]
                    dq[j] = p_Q[j][1]
                l.data[1][1][:, i] = q
                l.data[3][1][:, i] = dq
                l.data[5][1][:, i], l.data[7][1][:, i] = k.forward_kinematics(q, dq)

        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # logger
    if log:
        l.plot_channel([0, 1], int(T/dt), -1, 3, 4)
        l.plot_channel([2, 3], int(T/dt), -1, 3, 4)
        l.plot_channel([4, 5], int(T/dt), -1, 3, 4)
        l.plot_channel([6, 7], int(T/dt), -1, 3, 4)

    # quit pybullet
    p.disconnect()


def test_2():
    ####################
    #  INITIALIZATION ##
    ####################
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    k = BulletWrapper(L)

    duration = 3600  # define the duration of the simulation in seconds
    dt = 0.01  # define the time step in second
    robot_id, rev_joint_idx = configure_simulation(dt, True)
    Parameters.init_params()

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # compute desired configuration
        q = ParallelController.controller(k, *Parameters.get_params())

        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=q)

        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # quit pybullet
    p.disconnect()


def test3():
    T = 0.5
    dt = 0.01  # define the time step in second
    duration = 4000 * T  # define the duration of the simulation in seconds
    robot_id, rev_joint_idx = configure_simulation(dt, False)

    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()
        if dt * i < 3 * T:
            safeconfiguration(robot_id, rev_joint_idx)
        else:
            idleconfiguration(robot_id, rev_joint_idx)
        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep * 1)

    # quit pybullet
    p.disconnect()


if __name__ == '__main__':
    # test_1()
    # test_2()
    test3()
