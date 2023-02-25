import numpy as np

from solo_pybullet.model.robot.Kinematics import Kinematics


class BulletWrapper:
    def __init__(self, L):
        self.kinematics = Kinematics(L)

    def forward_kinematics(self, Q, dQ):
        # TODO
        pass

    def inverse_kinematics(self, P, dP, constraints):
        Q, dQ = self.kinematics.inverse_kinematics(P, dP, constraints)

        # convert model config to pybullet config
        for i in range(4):
            inv_X = -1 if i == 0 or i == 2 else 1
            inv_Y = 1 if i == 0 or i == 1 else -1

            Q[3 * i] = inv_X * (Q[3 * i] + np.pi / 2)
            Q[3 * i + 1] = inv_Y * Q[3 * i + 1]
            Q[3 * i + 2] = inv_Y * (Q[3 * i + 2] + np.pi)

            dQ[3 * i] = inv_X * dQ[3 * i]
            dQ[3 * i + 1] = inv_Y * dQ[3 * i + 1]
            dQ[3 * i + 2] = inv_Y * dQ[3 * i + 2]

        return Q, dQ
