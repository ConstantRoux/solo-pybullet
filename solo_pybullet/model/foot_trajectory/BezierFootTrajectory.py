import numpy as np
from matplotlib import pyplot as plt


class BezierFootTrajectory:
    @staticmethod
    def f(t, T, Pi, Pf):
        # modulo on a period
        t = t % T

        # scale t between 0 and 1
        t = t / T

        # swing
        if t < 1/2:
            x = Pi[0] * (1 - 2 * t) ** 3 + Pi[0] * 6 * t * (1 - 2 * t) ** 2 + Pf[0] * 12 * t ** 2 * (1 - 2 * t) + Pf[0] * 8 * t ** 3
            y = Pi[1] * (1 - 2 * t) ** 3 + Pi[1] * 6 * t * (1 - 2 * t) ** 2 + Pf[1] * 12 * t ** 2 * (1 - 2 * t) + Pf[1] * 8 * t ** 3
            if t < 1/4:
                z = Pi[2] * (1 - 4 * t) ** 3 + Pi[2] * 12 * t * (1 - 4 * t) ** 2 + Pf[2] * 48 * t ** 2 * (1 - 4 * t) + Pf[2] * 64 * t ** 3
            else:
                z = Pf[2] * (1 - 4 * (t-1/4)) ** 3 + Pf[2] * 12 * (t-1/4) * (1 - 4 * (t-1/4)) ** 2 + Pi[2] * 48 * (t-1/4) ** 2 * (1 - 4 * (t-1/4)) + Pi[2] * 64 * (t-1/4) ** 3
        # stance
        else:
            x = Pf[0] * (1 - 2 * (t-1/2)) ** 3 + Pf[0] * 6 * (t-1/2) * (1 - 2 * (t-1/2)) ** 2 + Pi[0] * 12 * (t-1/2) ** 2 * (1 - 2 * (t-1/2)) + Pi[0] * 8 * (t-1/2) ** 3
            y = Pf[1] * (1 - 2 * (t-1/2)) ** 3 + Pf[1] * 6 * (t-1/2) * (1 - 2 * (t-1/2)) ** 2 + Pi[1] * 12 * (t-1/2) ** 2 * (1 - 2 * (t-1/2)) + Pi[1] * 8 * (t-1/2) ** 3
            z = Pi[2]

        # return foot pos
        return np.array([x, y, z])


if __name__ == '__main__':
    T = 1
    t = np.linspace(0, T, 100)
    H = 0.05
    Lx = -0.1
    Ly = 0.1

    res = np.zeros((3, 100))

    for i, t0 in enumerate(t):
        res[:, i] = BezierFootTrajectory.f(t0, T, np.array([0, 0, -0.3]), np.array([0 + Lx, 0 + Ly, -0.3 + H]))

    plt.plot(t, res[0, :])
    plt.show()
    plt.plot(t, res[1, :])
    plt.show()
    plt.plot(t, res[2, :])
    plt.show()

