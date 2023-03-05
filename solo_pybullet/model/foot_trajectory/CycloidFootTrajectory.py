import numpy as np


class CycloidFootTrajectory:
    @staticmethod
    def f(t, T, X, H, L, dir=False):
        # modulo on a period
        t = t % T

        # way
        if dir:
            t = T - t

        # swing (hors sol)
        if t < T / 2:
            x = X[0] + 2 * L[0] * t / T
            y = X[1] + 2 * L[1] * t / T
            z = X[2] + H * np.sin(2 * np.pi * t / T)

        # stance (contact sol)
        else:
            x = X[0] + 2 * L[0] * (T - t) / T
            y = X[1] + 2 * L[1] * (T - t) / T
            z = X[2]

        return np.array([x, y, z])

    @staticmethod
    def df(t, T, H, L, dir=False):
        # modulo on a period
        t = t % T

        # way
        if dir:
            t = T - t

        # swing (hors sol)
        if t < T / 2:
            dx = 2 * L[0] / T
            dy = 2 * L[1] / T
            dz = (2 * H * np.pi / T) * np.cos(2 * np.pi * t / T)

        # stance (contact sol)
        else:
            dx = -2 * L[0] / T
            dy = -2 * L[1] / T
            dz = 0

        if dir:
            return np.array([-dx, -dy, -dz])
        else:
            return np.array([dx, dy, dz])
