#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from solo_pybullet.controller.initializer_controller.InitializerController import InitializerController
from solo_pybullet.simulation.initialization_simulation import configure_simulation
from solo_pybullet.model.robot.BulletWrapper import BulletWrapper


def test():
    """
    Test initialization function
    :return:
    """
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    T = 0.5
    dt = 0.01  # define the time step in second
    duration = 16 * T  # define the duration of the simulation in seconds
    robot_id, rev_joint_idx = configure_simulation(dt, False)
    constraints = np.array([0, np.pi, -np.pi, np.pi, -np.pi, 0] * 4)

    k = BulletWrapper(L)
    k.kinematics.debug = True

    safe_mode_period = 3
    safe_mode_flag = False

    wait_input_flag = True

    idle_mode_period = 3
    idle_time_start = None
    idle_height = 0.16
    idle_mode_flag = False

    testQ3 = []
    targetposQ3 = []

    # print(p.getJointInfo(robot_id, 2))

    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        if not safe_mode_flag:
            safe_mode_flag, q, dq = InitializerController.safe_configuration(k, i * dt, safe_mode_period, constraints)
            targetposQ3.append(q[2])  # keep the value to plot them (q3)

        elif wait_input_flag:  # wait for input to go in idle configuration
            x = input('Press the X key:')
            if x is 'x':
                wait_input_flag = False
                idle_time_start = dt * i

        elif not idle_mode_flag:
            idle_mode_flag, q, dq = InitializerController.idle_configuration(k, i * dt - idle_time_start, idle_mode_period, constraints, h=idle_height)
            targetposQ3.append(q[2])  # keep the value to plot them (q3)

        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=q, targetVelocities=dq)

        testQ3.append(p.getJointStates(robot_id, rev_joint_idx)[2][0])

        # next step simulation
        p.stepSimulation()

        # real time simulation
        t_sleep = dt - (time.perf_counter() - t0)
        if t_sleep > 0:
            time.sleep(t_sleep * 1)

    plt.plot(testQ3)
    plt.plot(targetposQ3)
    plt.show()

    # quit pybullet
    p.disconnect()


if __name__ == '__main__':
    test()
