import numpy as np

from solo_pybullet.model.robot.Kinematics import Kinematics


class BulletWrapper:
    def __init__(self, L):
        self.kinematics = Kinematics(L)

    def inverse_kinematics(self, P, dP, c):
        # TODO optimize
        q, dq = self.kinematics.inverse_kinematics(P, dP, c)

        # convert model config to pybullet config
        for i in range(4):
            way = 1 if i == 0 or i == 2 else -1
            q[3 * i] = way * (q[3 * i] + np.pi / 2)
            q[3 * i + 2] += np.pi

        for i in range(4):
            lateral = -1 if i == 0 or i == 2 else 1
            length = 1 if i == 0 or i == 1 else -1
            dq[3 * i] = lateral * dq[3 * i]
            dq[3 * i + 1] = length * dq[3 * i + 1]
            dq[3 * i + 2] = length * dq[3 * i + 2]

        return q, dq
