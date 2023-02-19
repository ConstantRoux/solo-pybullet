import numpy as np


def foot_trajectory(t, T, x0, y0, z0, H, L):
    # modulo on a period
    t = t % T

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
