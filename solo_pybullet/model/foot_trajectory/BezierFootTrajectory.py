import numpy as np
from matplotlib import pyplot as plt


class BezierFootTrajectory:
    @staticmethod
    def f(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # scale t between 0 and 1
        t = t / T

        # change dir
        if dir:
            t = 1 - t

        # swing
        if t < 1 / 2:
            x = Pi[0] * (1 - 2 * t) ** 3 + Pi[0] * 6 * t * (1 - 2 * t) ** 2 + Pf[0] * 12 * t ** 2 * (1 - 2 * t) + Pf[
                0] * 8 * t ** 3
            y = Pi[1] * (1 - 2 * t) ** 3 + Pi[1] * 6 * t * (1 - 2 * t) ** 2 + Pf[1] * 12 * t ** 2 * (1 - 2 * t) + Pf[
                1] * 8 * t ** 3
            if t < 1 / 4:
                z = Pi[2] * (1 - 4 * t) ** 3 + Pi[2] * 12 * t * (1 - 4 * t) ** 2 + Pf[2] * 48 * t ** 2 * (1 - 4 * t) + \
                    Pf[2] * 64 * t ** 3
            else:
                z = Pf[2] * (1 - 4 * (t - 1 / 4)) ** 3 + Pf[2] * 12 * (t - 1 / 4) * (1 - 4 * (t - 1 / 4)) ** 2 + Pi[2] * 48 * (t - 1 / 4) ** 2 * (1 - 4 * (t - 1 / 4)) + Pi[2] * 64 * (t - 1 / 4) ** 3
        # stance
        else:
            x = Pf[0] * (1 - 2 * (t - 1 / 2)) ** 3 + Pf[0] * 6 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2)) ** 2 + Pi[
                0] * 12 * (t - 1 / 2) ** 2 * (1 - 2 * (t - 1 / 2)) + Pi[0] * 8 * (t - 1 / 2) ** 3
            y = Pf[1] * (1 - 2 * (t - 1 / 2)) ** 3 + Pf[1] * 6 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2)) ** 2 + Pi[
                1] * 12 * (t - 1 / 2) ** 2 * (1 - 2 * (t - 1 / 2)) + Pi[1] * 8 * (t - 1 / 2) ** 3
            z = Pi[2]

        # return foot pos
        return np.array([x, y, z])

    @staticmethod
    def df(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # scale t between 0 and 1
        t = t / T

        # change dir
        if dir:
            t = 1 - t

        # swing
        if t < 1 / 2:
            dx = -6 * Pi[0] * (1 - 2 * t) ** 2 + 6 * Pi[0] * ((1 - 2 * t) ** 2 - 4 * t * (1 - 2 * t)) + 12 * Pf[0] * (
                        2 * t * (1 - 2 * t) - 2 * t ** 2) + 24 * Pf[0] * t ** 2
            dy = -6 * Pi[1] * (1 - 2 * t) ** 2 + 6 * Pi[1] * ((1 - 2 * t) ** 2 - 4 * t * (1 - 2 * t)) + 12 * Pf[1] * (
                        2 * t * (1 - 2 * t) - 2 * t ** 2) + 24 * Pf[1] * t ** 2
            if t < 1 / 4:
                dz = -12 * Pi[2] * (1 - 4 * t) ** 2 + 12 * Pi[2] * ((1 - 4 * t) ** 2 - 8 * t * (1 - 4 * t)) + 48 * Pf[
                    2] * (2 * t * (1 - 4 * t) - 4 * t ** 2) + 192 * Pf[2] * t ** 2
            else:
                dz = -12 * Pf[2] * (1 - 4 * (t - 1 / 4)) ** 2 + 12 * Pf[2] * (
                            (1 - 4 * (t - 1 / 4)) ** 2 - 8 * (t - 1 / 4) * (1 - 4 * (t - 1 / 4))) + 48 * Pi[2] * (
                                 2 * (t - 1 / 4) * (1 - 4 * (t - 1 / 4)) - 4 * (t - 1 / 4) ** 2) + 192 * Pi[2] * (
                                 t - 1 / 4) ** 2
        # stance
        else:
            dx = -6 * Pf[0] * (1 - 2 * (t - 1 / 2)) ** 2 + 6 * Pf[0] * (
                        (1 - 2 * (t - 1 / 2)) ** 2 - 4 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2))) + 12 * Pi[0] * (
                         2 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2)) - 2 * (t - 1 / 2) ** 2) + 24 * Pi[0] * (t - 1 / 2) ** 2
            dy = -6 * Pf[1] * (1 - 2 * (t - 1 / 2)) ** 2 + 6 * Pf[1] * (
                        (1 - 2 * (t - 1 / 2)) ** 2 - 4 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2))) + 12 * Pi[1] * (
                         2 * (t - 1 / 2) * (1 - 2 * (t - 1 / 2)) - 2 * (t - 1 / 2) ** 2) + 24 * Pi[1] * (t - 1 / 2) ** 2
            dz = 0

        # return foot pos
        if dir:
            return np.array([-dx, -dy, -dz])
        else:
            return np.array([dx, dy, dz])

    @staticmethod
    def d2f(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # scale t between 0 and 1
        t = t / T

        # change dir
        if dir:
            t = 1 - t

        # swing
        if t < 1 / 2:
            d2x = 24 * Pi[0] * (1 - 2 * t) + 6 * Pi[0] * (-4 * (1 - 2 * t) - 4 * (1 - 4 * t)) + 12 * Pf[0] * (2 * (1 - 4 * t) - 4 * t) + 48 * Pf[0] * t
            d2y = 24 * Pi[1] * (1 - 2 * t) + 6 * Pi[1] * (-4 * (1 - 2 * t) - 4 * (1 - 4 * t)) + 12 * Pf[1] * (2 * (1 - 4 * t) - 4 * t) + 48 * Pf[1] * t
            if t < 1 / 4:
                d2z = 96 * Pi[2] * (1 - 4 * t) - 96 * Pi[2] * ((1 - 4 * t) - 8 * (1 - 8 * t)) + 48 * Pf[2] * (2 * (1 - 8 * t) - 8 * t) + 384 * Pf[2] * t
            else:
                d2z = 96 * Pf[2] * (1 - 4 * (t - 1/4)) - 96 * Pf[2] * ((1 - 4 * (t - 1/4)) - 8 * (1 - 8 * (t - 1/4))) + 48 * Pi[2] * (2 * (1 - 8 * (t - 1/4)) - 8 * (t - 1/4)) + 384 * Pi[2] * (t - 1/4)
        # stance
        else:
            d2x = 24 * Pf[0] * (1 - 2 * (t - 1/2)) + 6 * Pf[0] * (-4 * (1 - 2 * (t - 1/2)) - 4 * (1 - 4 * (t - 1/2))) + 12 * Pi[0] * (2 * (1 - 4 * (t - 1/2)) - 4 * (t - 1/2)) + 48 * Pi[0] * (t - 1/2)
            d2y = 24 * Pf[1] * (1 - 2 * (t - 1/2)) + 6 * Pf[1] * (-4 * (1 - 2 * (t - 1/2)) - 4 * (1 - 4 * (t - 1/2))) + 12 * Pi[1] * (2 * (1 - 4 * (t - 1/2)) - 4 * (t - 1/2)) + 48 * Pi[1] * (t - 1/2)
            d2z = 0

        # return foot pos
        if dir:
            return np.array([-d2x, -d2y, -d2z])
        else:
            return np.array([d2x, d2y, d2z])


if __name__ == '__main__':
    n_size = 1000
    T = 1
    t = np.linspace(0, T, n_size)
    H = 0.05
    Lx = -0.1
    Ly = 0.1

    res = np.zeros((3, n_size))
    dres = np.zeros((3, n_size))
    d2res = np.zeros((3, n_size))

    for i, t0 in enumerate(t):
        res[:, i] = BezierFootTrajectory.f(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)
        dres[:, i] = BezierFootTrajectory.df(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)
        d2res[:, i] = BezierFootTrajectory.d2f(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)

    f, axs = plt.subplots(3, 3)
    axs[0, 0].plot(t, res[0, :])
    axs[1, 0].plot(t, dres[0, :])
    axs[2, 0].plot(t, d2res[0, :])
    axs[0, 1].plot(t, res[1, :])
    axs[1, 1].plot(t, dres[1, :])
    axs[2, 1].plot(t, d2res[1, :])
    axs[0, 2].plot(t, res[2, :])
    axs[1, 2].plot(t, dres[2, :])
    axs[2, 2].plot(t, d2res[2, :])

    axs[0, 0].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[1, 0].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[2, 0].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[0, 1].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[1, 1].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[2, 1].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[0, 2].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[1, 2].set_xlabel(r'$\frac{k.T_e}{T}$')
    axs[2, 2].set_xlabel(r'$\frac{k.T_e}{T}$')

    axs[0, 0].set_ylabel(r'x-axis position $\: [m]$')
    axs[1, 0].set_ylabel(r'x-axis velocity $\: [m/s]$')
    axs[2, 0].set_ylabel(r'x-axis acceleration $\: [m/s^2]$')
    axs[0, 1].set_ylabel(r'y-axis position $\: [m]$')
    axs[1, 1].set_ylabel(r'y-axis velocity $\: [m/s]$')
    axs[2, 1].set_ylabel(r'y-axis acceleration $\: [m/s^2]$')
    axs[0, 2].set_ylabel(r'z-axis position $\: [m]$')
    axs[1, 2].set_ylabel(r'z-axis velocity $\: [m/s]$')
    axs[2, 2].set_ylabel(r'z-axis acceleration $\: [m/s^2]$')

    plt.show()

