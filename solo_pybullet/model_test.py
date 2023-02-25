#####################
#  LOADING MODULES ##
#####################
import time

import numpy as np
import pybullet as p
from matplotlib import pyplot as plt

from solo_pybullet.initialization_simulation import configure_simulation
from solo_pybullet.model.foot_trajectory.cycloid_foot_trajectory import foot_trajectory, d_foot_trajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper

if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    dt = 0.01  # define the time step in second
    robot_id, rev_joint_idx = configure_simulation(dt, True)  # configure and load model in pybullet and pinocchio

    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    constraints = np.array([-np.pi, 0, 0, np.pi] * 4)
    k = BulletWrapper(L)
    T = 1
    Lp = 0.3
    x0 = -L[1] + L[2] + L[3] + L[5]
    y0 = -(Lp / 2 + L[0])
    z0 = -0.2
    H = 0.05

    duration = 4 * T  # define the duration of the simulation in seconds

    ####################
    #      LOGGER     ##
    ####################
    desired_q = np.zeros((12, int(duration / dt)))
    simulated_q = np.zeros((12, int(duration / dt)))
    desired_dq = np.zeros((12, int(duration / dt)))
    simulated_dq = np.zeros((12, int(duration / dt)))

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # get feet positions
        P = np.zeros((12,))
        P[0:3] = foot_trajectory(dt * i + T/2, T, x0, y0, z0, H, Lp, dir=True)
        P[3:6] = foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, dir=True)
        P[6:9] = foot_trajectory(dt * i + T/2, T, x0, y0, z0, H, Lp, dir=False)
        P[9:12] = foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, dir=False)

        dP = np.zeros((12,))
        dP[0:3] = d_foot_trajectory(dt * i + T/2, T, H, Lp, dir=True)
        dP[3:6] = d_foot_trajectory(dt * i, T, H, Lp, dir=True)
        dP[6:9] = d_foot_trajectory(dt * i + T/2, T, H, Lp, dir=False)
        dP[9:12] = d_foot_trajectory(dt * i, T, H, Lp, dir=False)

        # compute desired configuration
        q, dq = k.inverse_kinematics(P, dP, constraints)

        # logger
        desired_q[:, i] = q.copy()
        desired_dq[:, i] = dq.copy()

        # active actuators with new configuration
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=q,
                                    targetVelocities=dq
                                    )

        # next step simulation
        p.stepSimulation()

        # logger
        data = p.getJointStates(robot_id, rev_joint_idx)
        for j in range(12):
            q[j] = data[j][0]
            dq[j] = data[j][1]
        simulated_q[:, i] = q.copy()
        simulated_dq[:, i] = dq.copy()
        data2 = k.forward_kinematics(q, dq)

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # logger
    f, ax = plt.subplots(3, 4)
    for i in range(3):
        for k in range(4):
            ax[i, k].plot(desired_q[3 * k + i, int(duration / dt / 2):])
            ax[i, k].plot(simulated_q[3 * k + i, int(duration / dt / 2):])
    plt.show()

    f, ax = plt.subplots(3, 4)
    for i in range(3):
        for k in range(4):
            ax[i, k].plot(desired_dq[3 * k + i, int(duration / dt / 2):])
            ax[i, k].plot(simulated_dq[3 * k + i, int(duration / dt / 2):])
    plt.show()

    # quit pybullet
    p.disconnect()
