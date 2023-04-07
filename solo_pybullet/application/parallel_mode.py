
#####################
#  LOADING MODULES ##
#####################
import time
import numpy as np
import pybullet as p
from solo_pybullet.controller.parallel_controller.ParallelController import ParallelController
from solo_pybullet.controller.parallel_controller.Parameters import Parameters
from solo_pybullet.simulation.initialization_simulation import configure_simulation
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

    ###############
    #  MAIN LOOP ##
    ###############
    for i in range(int(duration / dt)):
        # real time simulation
        t0 = time.perf_counter()

        # compute desired configuration
        Q, dQ = ParallelController.controller(k, *Parameters.get_params(), constraints)
        
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=Q, targetVelocities=dQ)

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
