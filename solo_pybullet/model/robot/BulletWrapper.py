import matplotlib.pyplot as plt
import numpy as np
from math import remainder, tau
from solo_pybullet.model.robot.Kinematics import Kinematics


class BulletWrapper:
    def __init__(self, L):
        self.L = L
        self.kinematics = Kinematics(L)
        self.rev_counter = np.zeros((12,))

    def forward_kinematics(self, Q, dQ):
        for i in range(4):
            inv_X = -1 if i == 0 or i == 2 else 1
            inv_Y = 1 if i == 2 or i == 3 else -1

            Q[3 * i] = Q[3 * i] * inv_X + np.pi / 2
            Q[3 * i + 1] = Q[3 * i + 1] * inv_Y + np.pi
            Q[3 * i + 2] = Q[3 * i + 2] * inv_Y + np.pi

            dQ[3 * i] = dQ[3 * i] * inv_X
            dQ[3 * i + 1] = dQ[3 * i + 1] * inv_Y
            dQ[3 * i + 2] = dQ[3 * i + 2] * inv_Y

        return self.kinematics.forward_kinematics(Q, dQ)

    def body_inverse_kinematics(self, T, constraints):
        prev_Q = self.kinematics.current_Q
        Q = self.kinematics.body_inverse_kinematics(T, constraints)

        # convert model config to pybullet config
        self.rev_counter[(prev_Q - Q) >= np.pi] += 1
        self.rev_counter[(prev_Q - Q) <= -np.pi] -= 1
        Q += 2 * np.pi * self.rev_counter
        for i in range(4):
            inv_X = -1 if i == 0 or i == 2 else 1
            inv_Y = 1 if i == 2 or i == 3 else -1

            Q[3 * i] = inv_X * (Q[3 * i] - np.pi / 2)
            Q[3 * i + 1] = inv_Y * (Q[3 * i + 1] - np.pi)
            Q[3 * i + 2] = inv_Y * (Q[3 * i + 2] - np.pi)

        return Q

    def inverse_kinematics(self, P, dP, constraints):
        prev_Q = self.kinematics.current_Q
        Q, dQ = self.kinematics.inverse_kinematics(P, dP, constraints)

        # convert model config to pybullet config
        self.rev_counter[(prev_Q - Q) >= np.pi] += 1
        self.rev_counter[(prev_Q - Q) <= -np.pi] -= 1
        Q += 2 * np.pi * self.rev_counter
        for i in range(4):
            inv_X = -1 if i == 0 or i == 2 else 1
            inv_Y = 1 if i == 2 or i == 3 else -1

            Q[3 * i] = inv_X * (Q[3 * i] - np.pi / 2)
            Q[3 * i + 1] = inv_Y * (Q[3 * i + 1] - np.pi)
            Q[3 * i + 2] = inv_Y * (Q[3 * i + 2] - np.pi)

            dQ[3 * i] = inv_X * dQ[3 * i]
            dQ[3 * i + 1] = inv_Y * dQ[3 * i + 1]
            dQ[3 * i + 2] = inv_Y * dQ[3 * i + 2]

        return Q, dQ
