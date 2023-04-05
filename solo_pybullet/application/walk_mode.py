#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from solo_pybullet.controller.optimization_controller.OptimizationController import OptimizationController
from solo_pybullet.controller.walk_controller.WalkController import WalkController
from solo_pybullet.controller.walk_controller.Parameters import Parameters
from solo_pybullet.controller.initialization_controller.robot_initialization import idle_configuration, \
    safe_configuration
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.logger.Logger import Logger
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.foot_trajectory.BezierFootTrajectory import BezierFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper


def test_2():
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
        H = Parameters.get_params()[6]
        T = Parameters.get_params()[7]
        if T != 0:

            P[0:3] = CycloidFootTrajectory.f(dt * i + T / 2, T, np.array([x0, y0, z0]),
                                             np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=True)
            P[3:6] = CycloidFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]),
                                             dir=True)
            P[6:9] = CycloidFootTrajectory.f(dt * i + T / 2, T, np.array([x0, y0, z0]),
                                             np.array([x0 + Lpx, y0 + Lpy, z0 + H]), dir=False)
            P[9:12] = CycloidFootTrajectory.f(dt * i, T, np.array([x0, y0, z0]), np.array([x0 + Lpx, y0 + Lpy, z0 + H]),
                                              dir=False)

        else:
            P = np.array([-L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.,
                          -L[1] - L[2] - L[3] - L[5], -L[0], 0.])

        # compute desired configuration
        q = WalkController.controller(k, *Parameters.get_params()[:6], P, constraints)

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


if __name__ == '__main__':
    test_2()