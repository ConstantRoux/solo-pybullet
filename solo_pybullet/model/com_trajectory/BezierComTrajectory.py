import numpy as np


class BezierComTrajectory:
    @staticmethod
    def c(s, Pc):
        return (1 - s) * (1 - s) * Pc[0] + 2 * (1 - s) * s * Pc[1] + s * s * Pc[2]

    @staticmethod
    def tc(s, Pc):
        T = -2 * (1 - s) * Pc[0] + (2 - 4 * s) * Pc[1] + 2 * s * Pc[2]
        return T

    @staticmethod
    def nc(s, Pc):
        N = np.array([[0, -1], [1, 0]]) @ BezierComTrajectory.tc(s, Pc)
        return N
