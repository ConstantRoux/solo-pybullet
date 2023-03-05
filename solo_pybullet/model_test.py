#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
from matplotlib import pyplot as plt
from solo_pybullet.initialization_simulation import configure_simulation
from solo_pybullet.logger.Logger import Logger
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper


if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    constraints = np.array([0, np.pi, 0, np.pi] * 4)
    k = BulletWrapper(L)

    T = 1
    Lpx = -0.25
    Lpy = 0.15
    x0 = -L[1]
    y0 = -(L[0] + Lpy/2)
    z0 = -0.15
    H = 0.02

    duration = 4000 * T  # define the duration of the simulation in seconds
    dt = 0.01  # define the time step in second
    robot_id, rev_joint_idx = configure_simulation(dt, False)
    #      LOGGER     ##
    ####################
    log = False
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

        # get feet positions
        P = np.zeros((12,))
        P[0:3] = CycloidFootTrajectory.f(dt * i + T/2, T, np.array([x0, y0, z0]), H, np.array([Lpx, Lpy]), dir=False)
        P[3:6] = CycloidFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), H, np.array([Lpx, Lpy]), dir=False)
        P[6:9] = CycloidFootTrajectory.f((dt * i + T/2), T, np.array([x0, y0, z0]), H, np.array([Lpx, Lpy]), dir=True)
        P[9:12] = CycloidFootTrajectory.f((dt * i + T/2) + T/2, T, np.array([x0, y0, z0]), H, np.array([Lpx, Lpy]), dir=True)

        dP = np.zeros((12,))
        dP[0:3] = CycloidFootTrajectory.df(dt * i + T/2, T, H, np.array([Lpx, Lpy]), dir=False)
        dP[3:6] = CycloidFootTrajectory.df(dt * i, T, H, np.array([Lpx, Lpy]), dir=False)
        dP[6:9] = CycloidFootTrajectory.df((dt * i + T/2), T, H, np.array([Lpx, Lpy]), dir=True)
        dP[9:12] = CycloidFootTrajectory.df((dt * i + T/2) + T/2, T, H, np.array([Lpx, Lpy]), dir=True)

        # compute desired configuration
        q, dq = k.inverse_kinematics(P, dP, constraints)

        # logger
        if log:
            l.data[0][1][:, i] = q
            l.data[2][1][:, i] = dq
            l.data[4][1][:, i] = P
            l.data[6][1][:, i] = dP

        # active actuators with new configuration
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=q, targetVelocities=dq)

        # next step simulation
        p.stepSimulation()

        # logger
        if log:
            p_Q = p.getJointStates(robot_id, rev_joint_idx)
            for j in range(12):
                q[j] = p_Q[j][0]
                dq[j] = p_Q[j][1]
            l.data[1][1][:, i] = q
            l.data[3][1][:, i] = dq
            l.data[5][1][:, i], l.data[7][1][:, i] = k.forward_kinematics(q, dq)

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # logger
    if log:
        l.plot_channel([0, 1], int(duration / dt / 2), -1, 3, 4)
        l.plot_channel([2, 3], int(duration / dt / 2), -1, 3, 4)
        l.plot_channel([4, 5], int(duration / dt / 2), -1, 3, 4)
        l.plot_channel([6, 7], int(duration / dt / 2), -1, 3, 4)

    # quit pybullet
    p.disconnect()
