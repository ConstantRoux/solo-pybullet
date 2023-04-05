import numpy as np
from matplotlib import pyplot as plt


class BezierFootTrajectory:
    @staticmethod
    def f(t, T, Pi, Pf, dir=False):
        """
        Position trajectory
        :param t: time step
        :param T: Periode of a feet step
        :param Pi: Initial position
        :param Pf: Final position
        :param dir: Direction of the step
        :return:
        """
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