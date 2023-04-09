import threading

import numpy as np
from matplotlib import pyplot as plt

expected_speed_x = []
expected_speed_y = []
reached_speed_x = []
reached_speed_y = []


def logger_thread(dt, Vmax):
    f, ax = plt.subplots(2, 1)
    stp = int(5/dt)

    while True:
        ax[0].clear()
        ax[0].plot(expected_speed_x[len(expected_speed_x) - stp:], c='red')
        ax[0].plot(reached_speed_y[len(reached_speed_x) - stp:], c='blue')
        ax[1].clear()
        ax[1].plot(expected_speed_y[len(expected_speed_y) - stp:], c='red')
        ax[1].plot(reached_speed_x[len(reached_speed_y) - stp:], c='blue')

        ax[0].set_ylim([-Vmax * 1.5, Vmax * 1.5])
        ax[0].set_xlabel('Time [k.dt]')
        ax[0].set_ylabel('Velocity on x-axis [m/s]')

        ax[1].set_ylim([-Vmax * 1.5, Vmax * 1.5])
        ax[1].set_xlabel('Time [k.dt]')
        ax[1].set_ylabel('Velocity on y-axis [m/s]')

        ax[0].legend(['Expected velocity', 'Reached velocity'])
        ax[1].legend(['Expected velocity', 'Reached velocity'])

        f.tight_layout()
        plt.pause(0.001)
