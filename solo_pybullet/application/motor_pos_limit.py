#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt

from solo_pybullet.controller.optimization_controller.OptimizationController import OptimizationController
from solo_pybullet.controller.parallel_controller.ParallelController import ParallelController
from solo_pybullet.controller.parallel_controller.Parameters import Parameters
from solo_pybullet.controller.initialization_controller.robot_initialization import idle_configuration, safe_configuration
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.logger.Logger import Logger
from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
from solo_pybullet.model.foot_trajectory.BezierFootTrajectory import BezierFootTrajectory
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper

def test4():
    """
    Test q3 movement limit
    :return:
    """

    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    T = 0.3
    dt = 0.01  # define the time step in second
    duration = 4000 * T  # define the duration of the simulation in seconds
    robot_id, rev_joint_idx = configure_simulation(dt, True)
    k = BulletWrapper(L)
    k.kinematics.debug = True

    Q = np.zeros((12,))

    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # Q[2] = np.sin(i * dt)

        # print(q)
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=Q)  # targetVelocities=dq
        Q[2] += 0.01

        print(Q[2])

        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep * 1)

    # quit pybullet
    p.disconnect()

if __name__ == '__main__':
    test4()