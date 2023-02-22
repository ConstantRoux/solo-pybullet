import numpy as np


def foot_trajectory(t, T, x0, y0, z0, H, L, y=0):
    # modulo on a period
    t = t % T

    # way
    if y == 1:
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


def d_foot_trajectory(t, T, x0, y0, z0, H, L, y=0):
    # modulo on a period
    t = t % T

    # way
    if y == 1:
        t = T - t

    # swing (hors sol)
    if t < T / 2:
        x = 0
        y = 2 * L / T
        z = (2 * H * np.pi / T) * np.cos(2 * np.pi * t / T)

    # stance (contact sol)
    else:
        x = 0
        y = -2 * L / T
        z = 0

    return np.array([x, y, z])
