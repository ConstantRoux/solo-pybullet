#####################
#  LOADING MODULES ##
#####################
import threading

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
from solo_pybullet.controller.hybrid_controller.LoggerThread import logger_thread, expected_speed_x, expected_speed_y, reached_speed_x, reached_speed_y
from solo_pybullet.simulation.initialization_simulation import configure_simulation


def player():
    #################################
    #          PARAMETERS           #
    #################################
    # simulation parameters
    Vmax = 1

    #################################
    #            LOGGER             #
    #################################
    data = np.loadtxt('solo_pybullet/application/reader/movies/demo.csv', delimiter=',')
    dt = data[0, 0]

    #################################
    #       INITIALIZATION          #
    #################################
    # init simulation
    robot_id, rev_joint_idx = configure_simulation(dt, False)

    #################################
    #          SIMULATION           #
    #################################
    # local variables for simulation
    Q = np.zeros((12,))
    dQ = np.zeros((12,))

    #################################
    #           DEBUGGER            #
    #################################
    debug = False
    save_debug = True
    if debug:
        threading.Thread(target=logger_thread, args=(dt, Vmax)).start()

    print('start simulation...')
    for i in range(data.shape[0]):
        print(i/(data.shape[0]-1)*100, '%')
        if debug or save_debug:
            expected_speed_x.append(-data[i, 1])
            expected_speed_y.append(data[i, 2])
            lv, _ = p.getBaseVelocity(robot_id)
            reached_speed_x.append(lv[0])
            reached_speed_y.append(lv[1])

        #################################
        #        PD-CONTROL LOOP        #
        #################################
        p.setJointMotorControlArray(robot_id, rev_joint_idx, controlMode=p.POSITION_CONTROL,
                                    targetPositions=data[i, 3:15], targetVelocities=data[i, 15:27])

        #################################
        #           REAL TIME           #
        #################################
        # next step simulation
        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING, 1)
        p.stepSimulation()

        #################################
        #          SIMULATION           #
        #################################
        # uncomment to have autofocus camera on the robot
        basePos, baseOrn = p.getBasePositionAndOrientation(robot_id)  # Get model position
        eulerBasOrn = p.getEulerFromQuaternion(baseOrn)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=(i/4)%360, cameraPitch=-30,
                                     cameraTargetPosition=basePos)

    #################################
    #           DEBUGGER            #
    #################################
    if save_debug and not debug:
        print('start saving logger...')
        fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        out = cv2.VideoWriter("velocities.mp4", fourcc, 100, (1080, 720))
        f, ax = plt.subplots(2, 1, figsize=(7.5, 5))
        stp = int(5 / dt)
        for i in range(data.shape[0]):
            print(i / (data.shape[0] - 1)*100, '%')
            dp = i - stp if i - stp >= 0 else 0
            ax[0].clear()
            ax[0].plot(expected_speed_x[dp:i], c='red')
            ax[0].plot(reached_speed_y[dp:i], c='blue')
            ax[1].clear()
            ax[1].plot(expected_speed_y[dp:i], c='red')
            ax[1].plot(reached_speed_x[dp:i], c='blue')

            ax[0].set_ylim([-Vmax * 1.5, Vmax * 1.5])
            ax[0].set_xlabel('Time [k.dt]')
            ax[0].set_ylabel('Velocity on x-axis [m/s]')

            ax[1].set_ylim([-Vmax * 1.5, Vmax * 1.5])
            ax[1].set_xlabel('Time [k.dt]')
            ax[1].set_ylabel('Velocity on y-axis [m/s]')

            ax[0].legend(['Expected velocity', 'Reached velocity'])
            ax[1].legend(['Expected velocity', 'Reached velocity'])

            f.tight_layout()
            plt.pause(0.01)

            f_arr = np.array(f.canvas.renderer._renderer)
            f_arr = cv2.resize(f_arr, (1080, 720))
            # cv2.imshow('f_arr', f_arr) # Show f_arr for testing
            # cv2.waitKey(10)
            bgr = cv2.cvtColor(f_arr, cv2.COLOR_RGBA2BGR)
            out.write(bgr)
        out.release()

    # quit pybullet
    p.disconnect()


if __name__ == '__main__':
    player()
