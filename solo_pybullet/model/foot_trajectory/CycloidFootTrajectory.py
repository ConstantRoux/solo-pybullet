import numpy as np
from matplotlib import pyplot as plt


class CycloidFootTrajectory:
    @staticmethod
    def f(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # way
        if dir:
            t = T - t

        # swing (hors sol)
        if t < T / 2:
            x = Pi[0] + 2 * (Pf[0] - Pi[0]) * t / T
            y = Pi[1] + 2 * (Pf[1] - Pi[1]) * t / T
            z = Pi[2] + (Pf[2] - Pi[2]) * np.sin(2 * np.pi * t / T)

        # stance (contact sol)
        else:
            x = Pi[0] + 2 * (Pf[0] - Pi[0]) * (T - t) / T
            y = Pi[1] + 2 * (Pf[1] - Pi[1]) * (T - t) / T
            z = Pi[2]

        return np.array([x, y, z])

    @staticmethod
    def df(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # way
        if dir:
            t = T - t

        # swing (hors sol)
        if t < T / 2:
            dx = 2 * (Pf[0] - Pi[0]) / T
            dy = 2 * (Pf[1] - Pi[1]) / T
            dz = (2 * (Pf[2] - Pi[2]) * np.pi / T) * np.cos(2 * np.pi * t / T)

        # stance (contact sol)
        else:
            dx = -2 * (Pf[0] - Pi[0]) / T
            dy = -2 * (Pf[1] - Pi[1]) / T
            dz = 0

        if dir:
            return np.array([-dx, -dy, -dz])
        else:
            return np.array([dx, dy, dz])

    @staticmethod
    def d2f(t, T, Pi, Pf, dir=False):
        # modulo on a period
        t = t % T

        # way
        if dir:
            t = T - t

        # swing (hors sol)
        if t < T / 2:
            dx = 0
            dy = 0
            dz = -(Pf[2] - Pi[2]) * (2 * np.pi / T)**2 * np.sin(2 * np.pi * t / T)

        # stance (contact sol)
        else:
            dx = 0
            dy = 0
            dz = 0

        return np.array([dx, dy, dz])


if __name__ == '__main__':
    n_size = 10000
    T = 1
    t = np.linspace(0, T, n_size)
    H = 0.05
    Lx = 0.1
    Ly = 0.1

    res = np.zeros((3, n_size))
    dres = np.zeros((3, n_size))
    d2res = np.zeros((3, n_size))

    for i, t0 in enumerate(t):
        res[:, i] = CycloidFootTrajectory.f(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)
        dres[:, i] = CycloidFootTrajectory.df(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)
        d2res[:, i] = CycloidFootTrajectory.d2f(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]), dir=True)

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

    ax = plt.figure().add_subplot(projection='3d')
    ax.plot(res[0, :], res[1, :], res[2, :])
    plt.show()
