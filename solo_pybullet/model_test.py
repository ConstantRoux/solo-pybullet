#####################
#  LOADING MODULES ##
#####################
import time

import numpy as np
import pybullet as p
from solo_pybullet.initialization_simulation import configure_simulation
from solo_pybullet.model.foot_trajectory.cycloid_foot_trajectory import foot_trajectory, d_foot_trajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper

if __name__ == "__main__":
    ####################
    #  INITIALIZATION ##
    ####################
    dt = 0.01  # define the time step in second
    duration = 3600  # define the duration of the simulation in seconds
    robot_id, rev_joint_idx = configure_simulation(dt, False)  # configure and load model in pybullet and pinocchio

    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    constraints = np.array([-np.pi, 0, 0, np.pi] * 4)
    k = BulletWrapper(L)
    T = 0.5
    Lp = 0.3
    x0 = -L[1] + L[2] + L[3] + L[5]
    y0 = -(L[0] + Lp / 2)
    z0 = -0.2
    H = 0.1

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # get feet positions
        P = np.zeros((12,))
        P[0:3] =  foot_trajectory(dt * i + T/2, T, x0, y0, z0, H, Lp, y=1)
        P[3:6] =  foot_trajectory(dt * i + T/2, T, x0, y0, z0, H, Lp, y=1)
        P[6:9] =  foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, y=1)
        P[9:12] = foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, y=1)

        dP = np.zeros((12,))
        dP[0:3] =  d_foot_trajectory(dt * i + T / 2, T, x0, y0, z0, H, Lp, y=1)
        dP[3:6] =  d_foot_trajectory(dt * i + T / 2, T, x0, y0, z0, H, Lp, y=1)
        dP[6:9] =  d_foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, y=1)
        dP[9:12] = d_foot_trajectory(dt * i, T, x0, y0, z0, H, Lp, y=1)

        # compute desired configuration
        q, dq = k.inverse_kinematics(P, dP, constraints)

        # active actuators with new configuration
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
