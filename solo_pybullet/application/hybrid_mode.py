#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
from solo_pybullet.controller.hybrid_controller.HybridController import HybridController
from solo_pybullet.controller.hybrid_controller.Parameters import Parameters
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.model.foot_trajectory.BezierFootTrajectory import BezierFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper


def test():
    ####################
    #  INITIALIZATION ##
    ####################
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    k = BulletWrapper(L)
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)
    duration = 3600  # define the duration of the simulation in seconds
    dt = 0.01  # define the time step in second
    robot_id, rev_joint_idx = configure_simulation(dt, False)
    Parameters.init_params()

    T = 0.5
    Lpx = 0.
    Lpy = 0.1
    x0 = -L[1] - L[2] - L[3]
    y0 = -(L[0] + Lpy / 2)
    z0 = 0
    H = 0.05

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        P = np.zeros((12,))
        dP = np.zeros((12,))
        H = Parameters.get_params()[6]
        T = Parameters.get_params()[7]
        if T != 0:

            P[0:3] = BezierFootTrajectory.f(dt * i + T / 2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[3:6] = BezierFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[6:9] = BezierFootTrajectory.f(dt * i + T / 2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            P[9:12] = BezierFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

            dP[0:3] = BezierFootTrajectory.df(dt * i + T / 2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[3:6] = BezierFootTrajectory.df(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            dP[6:9] = BezierFootTrajectory.df(dt * i + T / 2, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            dP[9:12] = BezierFootTrajectory.df(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)

        else:
            P = np.array([-L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.])

            dP = np.zeros((12,))

        # compute desired configuration
        Q, dQ = HybridController.controller(k, *Parameters.get_params()[:6], P, dP, constraints)

        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL, targetPositions=Q, targetVelocities=dQ)

        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep)

    # quit pybullet
    p.disconnect()


if __name__ == '__main__':
    test()
