import numpy as np
from numpy import cos as c
from numpy import sin as s


class ParallelController:
    @staticmethod
    def R(rx, ry, rz):
        Rx = np.matrix([[1, 0, 0],
                        [0, c(rx), -s(rx)],
                        [0, s(rx), c(rx)]])
        Ry = np.matrix([[c(ry), 0, s(ry)],
                        [0, 1, 0],
                        [-s(ry), 0, c(ry)]])
        Rz = np.matrix([[c(rz), -s(rz), 0], [s(rz), c(rz), 0], [0, 0, 1]])
        return Rx @ Ry @ Rz

    @staticmethod
    def controller(k, tx, ty, tz, rx, ry, rz):
        """

        :param cmd: x, y, z orientations and translations of the base
        :return:
        """
        # set virtual initial robot shoulder pos in the FR leg frame
        v_FL = np.array([k.L[1], -k.L[0], 0.])
        v_FR = np.array([-k.L[1], -k.L[0], 0.])
        v_HL = np.array([k.L[1], k.L[0], 0.])
        v_HR = np.array([-k.L[1], k.L[0], 0.])

        # get desired rotation matrix
        R = ParallelController.R(rx, ry, rz)

        # get desired robot shoulder posin the FR leg frame
        d_FL = R @ v_FL
        d_FR = R @ v_FR
        d_HL = R @ v_HL
        d_HR = R @ v_HR
        d_FL[0, :] += np.array([tx, ty, tz])
        d_FR[0, :] += np.array([tx, ty, tz])
        d_HL[0, :] += np.array([tx, ty, tz])
        d_HR[0, :] += np.array([tx, ty, tz])

        # compute pos in each local frame
        delta_FL = d_FL @ k.kinematics.r['FL'][0:3, 0:3]
        delta_FR = d_FR @ k.kinematics.r['FR'][0:3, 0:3]
        delta_HL = d_HL @ k.kinematics.r['HL'][0:3, 0:3]
        delta_HR = d_HR @ k.kinematics.r['HR'][0:3, 0:3]

        # compute configurations
        P = np.array(np.concatenate([delta_FL[0, :], delta_FR[0, :], delta_HL[0, :], delta_HR[0, :]], axis=1)).flatten()
        dP = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        q, dq = k.inverse_kinematics(P, dP, np.array([0, np.pi, 0, np.pi] * 4))

        return q, dq