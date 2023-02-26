import numpy as np


def foot_trajectory(t, T, x0, y0, z0, H, L, dir=False):
    # modulo on a period
    t = t % T

    # way
    if dir:
        t = T - t

    # swing (hors sol)
    if t < T / 2:
        x = x0
        y = y0 + 2 * L * t / T
        z = z0 + H * np.sin(2 * np.pi * t / T)

    # stance (contact sol)
    else:
        x = x0
        y = y0 + 2 * L * (T - t) / T
        z = z0

    return np.array([x, y, z])


def d_foot_trajectory(t, T, H, L, dir=False):
    # modulo on a period
    t = t % T

    # way
    if dir:
        t = T - t

    # swing (hors sol)
    if t < T / 2:
        dx = 0
        dy = 2 * L / T
        dz = (2 * H * np.pi / T) * np.cos(2 * np.pi * t / T)

    # stance (contact sol)
    else:
        dx = 0
        dy = -2 * L / T
        dz = 0

    if dir:
        return np.array([-dx, -dy, -dz])
    else:
        return np.array([dx, dy, dz])
