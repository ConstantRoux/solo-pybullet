#####################
#  LOADING MODULES ##
#####################
import numpy as np

T = 0.5  # period of the foot trajectory

xF0 = 0.207  # initial X position of the front feet
xH0 = -0.207  # initial X position of the hind feet

yF0 = 0.14695  # initial Y position of the front feet
yH0 = -0.14695  # initial Y position of the hind feet

z0 = -0.25  # negative distance between the base and the ground

dx = 0.03  # displacement amplitude by x
dz = 0.03  # displacement amplitude by z


def foot_trajectory(t, x0, y0, z0):  # arguments : time, initial position x and z
    x = []
    y = []
    z = []
    if t >= T:
        t %= T
    x.append(x0 - dx * np.cos(2 * np.pi * t / T))
    if t <= T / 2.:
        z.append(z0 + dz * np.sin(2 * np.pi * t / T))
    else:
        z.append(z0)
    y.append(y0)
    return np.matrix([x, y, z, [1]])
