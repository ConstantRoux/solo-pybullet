import numpy as np
from numpy import cos as c
from numpy import sin as s


class Kinematics:
    def __init__(self, L):
        self.L = L

    def forward_kinematics(self, Q):
        q1, q2, q3 = Q
        L1, L2, L3, L4, L5, L6, L7 = self.L
        return np.matrix([[c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3),  -c(q1) * c(q2) * s(q3) - c(q1) * c(q3) * s(q2), s(q1), -L7 * (c(q1) * c(q2) * c(q3) - c(q1) * s(q2) * s(q3)) - L6 * s(q1) + c(q1) * c(q2) * L5 - L4 * s(q1) - L2],
                          [-c(q3) * s(q2) - c(q2) * s(q3),  s(q2) * s(q3) - c(q2) * c(q3), 0, L7 * (c(q3) * s(q2) + c(q2) * s(q3)) - s(q2) * L5 - (L1 + L3)],
                          [s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3),  -s(q1) * c(q2) * s(q3) - s(q1) * c(q3) * s(q2), -c(q1), -L7 * (s(q1) * c(q2) * c(q3) - s(q1) * s(q2) * s(q3)) + L6 * c(q1) + s(q1) * c(q2) * L5 + L4 * c(q1)],
                          [0, 0, 0, 1]])

    def inverse_kinematics(self, T):
        L = self.L
        q1 = np.arctan2(T[0, 2], -T[2, 2])
        q23 = np.arctan2(-T[1, 0], T[0, 0] / np.cos(q1))
        q2 = np.arctan2((T[1, 3] + L[0] + L[2] + L[6] * T[1, 0]) / -L[4],
                        (T[2, 3] + L[6] * np.sin(q1) * np.cos(q23) - np.cos(q1) * (L[3] + L[5])) / (np.sin(q1) * L[4]))
        q3 = q23 - q2
        return np.array([q1, q2, q3])


if __name__ == "__main__":
    k = Kinematics(np.array([0.1, 0.1, 0.15, 0.03, 0.15, 0.03, 0.15]))
    q = np.array([.45, 0, -0.5])
    t = k.forward_kinematics(q)
    print(k.inverse_kinematics(t))

    print(k.inverse_kinematics(np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -0.15], [0, 0, 0, 1]])))
