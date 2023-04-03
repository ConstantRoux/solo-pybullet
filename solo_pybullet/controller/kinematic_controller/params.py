import pybullet as p


T = 0.5  # period of the foot trajectory

xF0 = 0.207  # initial X position of the front feet
xH0 = -0.207  # initial X position of the hind feet

yF0 = 0.14695  # initial Y position of the front feet
yH0 = -0.14695  # initial Y position of the hind feet

z0 = -0.25  # negative distance between the base and the ground

dx = 0.03  # displacement amplitude by x
dz = 0.03  # diIntializesplacement amplitude by z


def init_params():
    """
    Intialize slider for the static mode, for test only
    :return:
    """
    global T, xF0, yF0, z0, dx, dz
    p.addUserDebugParameter("  T  ",  0, 10, T)
    p.addUserDebugParameter("  x0  ", 0, 0.35, xF0)
    p.addUserDebugParameter("  y0  ", 0, 0.35, yF0)
    p.addUserDebugParameter("  z0  ", -0.33, 0, z0)
    p.addUserDebugParameter("  dx  ", 0, 0.2, dx)
    p.addUserDebugParameter("  dz  ", 0, 0.2, dz)


def update_params():
    """
    Update slider for the static mode, for test only
    :return:
    """
    global T, xF0, xH0, yF0, yH0, z0, dx, dz
    T = p.readUserDebugParameter(0)
    xF0 = p.readUserDebugParameter(1)
    xH0 = -xF0
    yF0 = p.readUserDebugParameter(2)
    yH0 = -yF0
    z0 = p.readUserDebugParameter(3)
    dx = p.readUserDebugParameter(4)
    dz = p.readUserDebugParameter(5)

    return T, xF0, xH0, yF0, yH0, z0, dx, dz