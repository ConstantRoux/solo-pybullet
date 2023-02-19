import numpy as np
from numpy import cos as c
from numpy import sin as s


# TODO kinematics front left -> front right -> h left -> h right
class Kinematics:
    def __init__(self, L):
        # offsets
        self.L = L

        # rotation to pass from FR to others
        self.labels = ['FL', 'FR', 'BR', 'BL']
        self.r = {'FL': np.matrix([[-1, 0, 0, 0],
                                    [0, 1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]]),
                  'FR': np.matrix([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'BR': np.matrix([[1, 0, 0, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'BL': np.matrix([[-1, 0, 0, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 1, 0],
                                    [0, 0, 0, 1]])}

        # self.L[0] = 0
        # self.L[1] = 0

    def forward_kinematics(self, Q):
        list = []
        for i in range(4):
            list.append(self.__forward_kinematics(Q[3 * i: 3 * (i + 1)], self.r[self.labels[i]]))
        return list

    def inverse_kinematics(self, pos):
        Q = np.zeros((12,))
        for i in range(len(pos)):
            Q[3 * i: 3 * (i + 1)] = self.__inverse_kinematics(pos[i], self.r[self.labels[i]])
        return Q

    def __forward_kinematics(self, Q, R):
        q1, q2, q3 = Q
        L1, L2, L3, L4, L5, L6, L7 = self.L
        T04 = np.matrix([[c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3),
                           -c(q1) * c(q2) * s(q3) - c(q1) * c(q3) * s(q2), -s(q1),
                           -L7 * (c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3)) - L6 * s(q1) + c(q1) * c(
                               q2) * L5 - L4 * s(q1) - L2],
                          [c(q3) * s(q2) + c(q2) * s(q3), -s(q2) * s(q3) + c(q2) * c(q3), 0,
                           -L7 * (c(q3) * s(q2) + c(q2) * s(q3)) + s(q2) * L5 - (L1 + L3)],
                          [s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3),
                           -s(q1) * c(q2) * s(q3) - s(q1) * c(q3) * s(q2), c(q1),
                           -L7 * (s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3)) + L6 * c(q1) + s(q1) * c(
                               q2) * L5 + L4 * c(q1)],
                          [0, 0, 0, 1]])

        return R * T04

    def __inverse_kinematics(self, pos, R):
        L1, L2, L3, L4, L5, L6, L7 = self.L

        # compute q1
        X = pos[2]
        Y = -pos[0] - L2
        Z = L4 + L6
        q1 = np.arctan2((Y * Z + X * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y),
                        (X * Z - Y * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y))

        # compute q3
        Z1 = (pos[2] - np.cos(q1) * (L4 + L6)) / np.sin(q1)
        Z2 = pos[1] + L1 + L3
        c3 = (Z1 * Z1 + Z2 * Z2 - L5 * L5 - L7 * L7) / (-2 * L5 * L7)
        q3_1 = np.arctan2(np.sqrt(1 - c3 * c3), c3)
        q3_2 = np.arctan2(-np.sqrt(1 - c3 * c3), c3)

        # compute q2
        B1_1 = L5 - L7 * np.cos(q3_1)
        B1_2 = L5 - L7 * np.cos(q3_2)
        B2_1 = -L7 * np.sin(q3_1)
        B2_2 = -L7 * np.sin(q3_2)
        q2_1 = np.arctan2((B1_1 * Z2 - B2_1 * Z1) / (B1_1 * B1_1 + B2_1 * B2_1),
                          (B1_1 * Z1 + B2_1 * Z2) / (B1_1 * B1_1 + B2_1 * B2_1))
        q2_2 = np.arctan2((B1_2 * Z2 - B2_2 * Z1) / (B1_2 * B1_2 + B2_2 * B2_2),
                          (B1_2 * Z1 + B2_2 * Z2) / (B1_2 * B1_2 + B2_2 * B2_2))

        return np.array([q1, q2_1, q3_1])

    def __T01(self, q1):
        L = self.L
        return np.matrix([[c(q1), -s(q1), 0, -L[1]],
                          [0, 0, -1, -(L[0] + L[2])],
                          [s(q1), c(q1), 0, 0],
                          [0, 0, 0, 1]])

    def __T12(self, q2):
        return np.matrix([[c(q2), -s(q2), 0, 0],
                          [0, 0, 1, 0],
                          [-s(q2), -c(q2), 0, 0],
                          [0, 0, 0, 1]])

    def __T23(self, q3):
        L = self.L
        return np.matrix([[c(q3), -s(q3), 0, L[4]],
                          [s(q3), c(q3), 0, 0],
                          [0, 0, 1, L[3]],
                          [0, 0, 0, 1]])

    def __T34(self):
        L = self.L
        return np.matrix([[1, 0, 0, -L[6]],
                          [0, 1, 0, 0],
                          [0, 0, 1, L[5]],
                          [0, 0, 0, 1]])

    def T0k(self, q, k, R):
        funcs = [self.__T01, self.__T12, self.__T23, self.__T34]
        T0k = np.identity(4)
        for i in range(0, k):
            if i == 3:
                T0k = np.dot(T0k, funcs[i]())
            else:
                T0k = np.dot(T0k, funcs[i](q[i]))
        return R * T0k


def test_1():
    # imports
    from solo_pybullet.model.foot_trajectory.cycloid_foot_trajectory import foot_trajectory
    from solo_pybullet.model.robot.Viewer import Viewer

    # variables
    L = [0.15, 0.1, 0.15, 0.03, 0.15, 0.03, 0.15]
    T = 1
    Lp = 0.15
    x0 = -(L[1] + L[3] + L[5])
    y0 = -(L[0] + L[2] - Lp / 2)
    z0 = -0.2
    H = 0.05
    kinematics = Kinematics(L)

    # test forward kinematics
    Viewer.viewForwardKinematics(kinematics)

    # test inverse kinematics
    # get space points and orientation of end effector
    t = np.linspace(0, T, 10)
    res = np.empty((12, len(t)))
    for i, t0 in enumerate(t):
        res[0:3, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[3:6, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[6:9, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[9:12, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)

    Viewer.viewInverseKinematics(kinematics, res)


if __name__ == '__main__':
    test_1()
