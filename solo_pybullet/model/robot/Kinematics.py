import numpy as np
from numpy import cos as c
from numpy import sin as s


class Kinematics:
    def __init__(self, L):
        # offsets
        self.L = L

        # rotation to pass from FR to others
        self.labels = ['FL', 'FR', 'HL', 'HR']
        self.r = {'FL': np.matrix([[-1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'FR': np.matrix([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'HL': np.matrix([[1, 0, 0, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'HR': np.matrix([[-1, 0, 0, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])}

    def forward_kinematics(self, Q, dQ):
        P = np.zeros(12, )
        dP = np.zeros(12, )

        for i in range(4):
            P[3 * i: 3 * (i + 1)] = self.__forward_kinematics(Q[3 * i: 3 * (i + 1)], self.r[self.labels[i]])
            dP[3 * i: 3 * (i + 1)] = self.__linearJacobian(Q[3 * i: 3 * (i + 1)]) @ dQ[3 * i: 3 * (i + 1)]

        return P, dP

    def inverse_kinematics(self, P, dP, constraints):
        Q = np.zeros((12,))
        dQ = np.zeros((12,))

        for i in range(4):
            Q[3 * i: 3 * (i + 1)] = self.__inverse_kinematics(P[3 * i: 3 * (i + 1)], constraints[4 * i: 4 * (i + 1)])
            dQ[3 * i: 3 * (i + 1)] = np.linalg.pinv(self.__linearJacobian(Q[3 * i: 3 * (i + 1)])) @ dP[3 * i: 3 * (i + 1)]
        return Q, dQ

    def linearJacobian(self, Q):
        J = np.zeros((3, 12))
        for i in range(4):
            J[:, 3 * i: 3 * (i + 1)] = self.__linearJacobian(Q[3 * i: 3 * (i + 1)])
        return J

    def __forward_kinematics(self, Q, R):
        q1, q2, q3 = Q
        L1, L2, L3, L4, L5, L6, L7 = self.L
        T04 = np.matrix([[c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3),
                          -c(q1) * c(q2) * s(q3) - c(q1) * c(q3) * s(q2), -s(q1),
                          -L7 * (c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3)) - L6 * s(q1) + c(q1) * c(
                              q2) * L5 - (L3 + L4) * s(q1) - L2],
                         [c(q3) * s(q2) + c(q2) * s(q3), -s(q2) * s(q3) + c(q2) * c(q3), 0,
                          -L7 * (c(q3) * s(q2) + c(q2) * s(q3)) + s(q2) * L5 - L1],
                         [s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3),
                          -s(q1) * c(q2) * s(q3) - s(q1) * c(q3) * s(q2), c(q1),
                          -L7 * (s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3)) + L6 * c(q1) + s(q1) * c(
                              q2) * L5 + (L3 + L4) * c(q1)],
                         [0, 0, 0, 1]])

        return (R * T04)[0:3, 3].flatten()

    def __inverse_kinematics(self, pos, constraints):
        # TODO check if robot is able to reach the desired position
        # TODO optimize calculus
        # TODO constraint q2 joint + exception if no solution has been found
        L1, L2, L3, L4, L5, L6, L7 = self.L

        # compute q1
        X = pos[2]
        Y = -pos[0] - L2
        Z = L3 + L4 + L6
        q1 = np.arctan2((Y * Z - X * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y),
                        (X * Z + Y * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y))
        if q1 < constraints[0] or q1 > constraints[1]:
            q1 = np.arctan2((Y * Z + X * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y),
                            (X * Z - Y * np.sqrt(X * X + Y * Y - Z * Z)) / (X * X + Y * Y))

        # compute q3
        Z1 = (pos[2] - np.cos(q1) * (L3 + L4 + L6)) / np.sin(q1)
        Z2 = pos[1] + L1
        c3 = (Z1 * Z1 + Z2 * Z2 - L5 * L5 - L7 * L7) / (-2 * L5 * L7)
        q3 = np.arctan2(np.sqrt(1 - c3 * c3), c3)
        if q3 < constraints[2] or q3 > constraints[3]:
            q3 = np.arctan2(-np.sqrt(1 - c3 * c3), c3)

        # compute q2
        B1 = L5 - L7 * np.cos(q3)
        B2 = -L7 * np.sin(q3)
        q2_1 = np.arctan2((B1 * Z2 - B2 * Z1) / (B1 * B1 + B2 * B2),
                          (B1 * Z1 + B2 * Z2) / (B1 * B1 + B2 * B2))

        return np.array([q1, q2_1, q3])

    def __linearJacobian(self, Q):
        q1, q2, q3 = Q
        L1, L2, L3, L4, L5, L6, L7 = self.L

        return np.matrix([[c(q1) * (-L3 - L4 - L6) - s(q1) * (c(q2) * L5 - L7 * c(q2 + q3)),
                           c(q1) * (L7 * s(q2 + q3) - s(q2) * L5), c(q1) * L7 * s(q2 + q3)],
                          [0, c(q2) * L5 - L7 * c(q2 + q3), -L7 * c(q2 + q3)],
                          [s(q1) * (-L3 - L4 - L6) + c(q1) * (c(q2) * L5 - L7 * c(q2 + q3)),
                           s(q1) * (L7 * s(q2 + q3) - s(q2) * L5), s(q1) * L7 * s(q2 + q3)]])

    def __T01(self, q1):
        L = self.L
        return np.matrix([[c(q1), -s(q1), 0, -L[1]],
                          [0, 0, -1, -L[0]],
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
                          [0, 0, 1, L[2] + L[3]],
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
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    T = 1
    Lp = 0.15
    x0 = -L[1]
    y0 = -(L[0] + Lp / 2)
    z0 = -0.2
    H = 0.05
    kinematics = Kinematics(L)

    # test forward kinematics
    Viewer.viewForwardKinematics(kinematics)

    # test inverse kinematics
    # get space points and orientation of end effector
    t = np.linspace(0, T, 15)
    res = np.empty((12, len(t)))
    for i, t0 in enumerate(t):
        res[0:3, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[3:6, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[6:9, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)
        res[9:12, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)

    Viewer.viewInverseKinematics(kinematics, res)


def test_2():
    # imports
    from solo_pybullet.model.foot_trajectory.cycloid_foot_trajectory import foot_trajectory, d_foot_trajectory

    # variables
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    T = 1
    Lp = 0.15
    x0 = -L[1]
    y0 = -(L[0] + Lp / 2)
    z0 = -0.2
    H = 0.05
    kinematics = Kinematics(L)
    constraints = np.array([0, np.pi, 0, np.pi] * 4)

    # test inverse kinematics
    # get space points and orientation of end effector
    t = np.linspace(0, 2 * T, 200)
    res = np.empty((12, len(t)))
    d_res = np.empty((12, len(t)))

    for i, t0 in enumerate(t):
        res[0:3, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[3:6, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        res[6:9, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)
        res[9:12, i] = foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)

        d_res[0:3, i] = d_foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        d_res[3:6, i] = d_foot_trajectory(t0, T, x0, y0, z0, H, Lp)
        d_res[6:9, i] = d_foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)
        d_res[9:12, i] = d_foot_trajectory(t0, T, x0, y0, z0, H, Lp, y=1)

        q, dq = kinematics.inverse_kinematics(res[:, i], d_res[:, i], constraints)

        pos, d_pos = kinematics.forward_kinematics(q, dq)

        print(np.linalg.norm(d_pos[3:6]-d_res[3:6, i]))


if __name__ == '__main__':
    test_1()
    # test_2()
