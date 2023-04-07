import numpy as np
from numpy import cos as c
from numpy import sin as s

from solo_pybullet.exception.NoSolutionException import NoSolutionException
from solo_pybullet.exception.NotReachableException import NotReachableException
from solo_pybullet.model.foot_trajectory.BezierFootTrajectory import BezierFootTrajectory


class Kinematics:
    def __init__(self, L):
        """

        :param L: length between links
        """
        self.L = L

        # rotation to pass from FR (FrontRight) leg to other legs
        self.labels = ['FL', 'FR', 'HL', 'HR']
        self.r = {'FL': np.matrix([[-1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'FR': np.matrix([[1, 0, 0, 0],
                                   [0, 1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'HL': np.matrix([[-1, 0, 0, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]),
                  'HR': np.matrix([[1, 0, 0, 0],
                                   [0, -1, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])}

        # current config of the robot
        self.current_Q = np.array([np.pi/2, np.pi, np.pi] * 4)

        # debug parameter to display information and error messages
        self.debug = False

    def forward_kinematics(self, Q, dQ):
        """
        compute the robot position form the current configuration
        :param Q: current configuration
        :param dQ: current velocities of joints
        :return:
        """
        P = np.zeros(12, )
        dP = np.zeros(12, )

        for i in range(4):
            P[3 * i: 3 * (i + 1)] = self.__forward_kinematics(Q[3 * i: 3 * (i + 1)])
            dP[3 * i: 3 * (i + 1)] = self.__linearJacobian(Q[3 * i: 3 * (i + 1)]) @ dQ[3 * i: 3 * (i + 1)]

        return P, dP

    def inverse_kinematics(self, P, dP, constraints):
        """
        Compute the robot joints configuration from points in space
        :param P: Points in space of each leg, [x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4]
        :param dP: Velocities in space of each leg, [dx1, dy1, dz1, dx2, dy2, dz2, dx3, dy3, dz3, dx4, dy4, dz4]
        :param constraints: constrain the robot joints
        :return: return the current configuration and the velocities of each joints
        """
        Q = np.zeros((12,))
        dQ = np.zeros((12,))

        try:
            for i in range(4):
                Q[3 * i: 3 * (i + 1)] = self.__inverse_kinematics(P[3 * i: 3 * (i + 1)], constraints[6 * i: 6 * (i + 1)], i)
                dQ[3 * i: 3 * (i + 1)] = np.linalg.pinv(self.__linearJacobian(Q[3 * i: 3 * (i + 1)])) @ dP[3 * i: 3 * (i + 1)]
        except (NoSolutionException, NotReachableException) as err:
            if self.debug:
                print(err)
            Q = np.copy(self.current_Q)
        else:
            self.current_Q = np.copy(Q)

        return Q, dQ

    def body_inverse_kinematics(self, T, P, dP, constraints):
        """

        :param T: Homogeneous matrix giving rotation and translation
        :param P:
        :param dP:
        :param constraints: constrain the robot joints
        :return: Configuration of each robot links
        """
        # compute T10 (shoulder to foot)
        Tinv = np.linalg.inv(T)

        # convert feet position expressed in local frame in world frame
        FL0 = self.r['FL'] @ np.append(P[0:3], 1)
        FR0 = self.r['FR'] @ np.append(P[3:6], 1)
        HL0 = self.r['HL'] @ np.append(P[6:9], 1)
        HR0 = self.r['HR'] @ np.append(P[9:12], 1)

        # get foot positions in the shoulder frame
        FL1 = Tinv @ FL0.T
        FR1 = Tinv @ FR0.T
        HL1 = Tinv @ HL0.T
        HR1 = Tinv @ HR0.T

        # compute pos in each local frame
        local_FL = (self.r['FL'] @ FL1).T
        local_FR = (self.r['FR'] @ FR1).T
        local_HL = (self.r['HL'] @ HL1).T
        local_HR = (self.r['HR'] @ HR1).T

        # convert feet velocity expressed in local frame in world frame
        dFL0 = self.r['FL'] @ np.append(dP[0:3], 1)
        dFR0 = self.r['FR'] @ np.append(dP[3:6], 1)
        dHL0 = self.r['HL'] @ np.append(dP[6:9], 1)
        dHR0 = self.r['HR'] @ np.append(dP[9:12], 1)

        # get foot velocities in the shoulder frame
        dFL1 = Tinv @ dFL0.T
        dFR1 = Tinv @ dFR0.T
        dHL1 = Tinv @ dHL0.T
        dHR1 = Tinv @ dHR0.T

        # compute pos in each local frame
        local_dFL = (self.r['FL'] @ dFL1).T
        local_dFR = (self.r['FR'] @ dFR1).T
        local_dHL = (self.r['HL'] @ dHL1).T
        local_dHR = (self.r['HR'] @ dHR1).T

        # compute configurations
        P = np.array(np.concatenate([local_FL[0, :3], local_FR[0, :3], local_HL[0, :3], local_HR[0, :3]], axis=1)).flatten()
        dP = np.array(np.concatenate([local_dFL[0, :3], local_dFR[0, :3], local_dHL[0, :3], local_dHR[0, :3]], axis=1)).flatten()
        Q, dQ = self.inverse_kinematics(P, dP, constraints)

        return Q, dQ

    def linearJacobian(self, Q):
        """
        Compute the Jacobian matrix for the entire robot
        :param Q: current joints configuration
        :return: Jacobian matrix
        """
        J = np.zeros((3, 12))
        for i in range(4):
            J[:, 3 * i: 3 * (i + 1)] = self.__linearJacobian(Q[3 * i: 3 * (i + 1)])
        return J

    def __forward_kinematics(self, Q):
        """
        Compute the leg position in relation to robot frame
        :param Q: Current configuration of a leg
        :return: Leg position in the robot frame
        """
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

        return T04[0:3, 3].flatten()

    def __inverse_kinematics(self, pos, constraints, leg_idx):
        """
        Compute the configuration of a leg from it position in relation to the robot frame
        :param pos: Point in the robot frame
        :param constraints: constrain the robot joints
        :param leg_idx: which leg is consdered
        :return: Leg configuration
        """
        L1, L2, L3, L4, L5, L6, L7 = self.L

        ######################################################
        #                   compute q1                       #
        ######################################################
        X = pos[2]
        Y = -pos[0] - L2
        Z = L3 + L4 + L6

        sqrt_tmp = np.sqrt(X * X + Y * Y - Z * Z)
        if np.isnan(sqrt_tmp):
            raise NotReachableException(pos)
        q1 = np.arctan2((Y * Z - X * sqrt_tmp) / (X * X + Y * Y),
                        (X * Z + Y * sqrt_tmp) / (X * X + Y * Y))
        if q1 < constraints[0] or q1 > constraints[1]:
            q1 = np.arctan2((Y * Z + X * sqrt_tmp) / (X * X + Y * Y),
                            (X * Z - Y * sqrt_tmp) / (X * X + Y * Y))
            if q1 < constraints[0] or q1 > constraints[1]:
                raise NoSolutionException(pos, constraints)

        ######################################################
        #                   compute q3                       #
        ######################################################
        # case q1 is near zero -> singularity
        if q1 == 0 or np.abs(q1) == np.pi:
            Z1 = pos[0] + L2
        else:
            Z1 = (pos[2] - c(q1) * (L3 + L4 + L6)) / s(q1)
        Z2 = pos[1] + L1
        c3 = (Z1 * Z1 + Z2 * Z2 - L5 * L5 - L7 * L7) / (-2 * L5 * L7)
        sqrt_tmp = np.sqrt(1 - c3 * c3)
        if np.isnan(sqrt_tmp):
            raise NotReachableException(pos)
        q3 = np.arctan2(sqrt_tmp, c3)
        if q3 < constraints[4] or q3 > constraints[5]:
            q3 = np.arctan2(-sqrt_tmp, c3)
            if q3 < constraints[4] or q3 > constraints[5]:
                raise NoSolutionException(pos, constraints)

        ######################################################
        #                   compute q2                       #
        ######################################################
        # case q3 is near zero -> infinity of solutions
        if q3 == 0:
            q2 = self.current_Q[3 * leg_idx + 1]

        # case q3 is near pi
        elif np.abs(q3) == np.pi:
            q2 = np.arctan2((pos[1] + L1)/(L5 + L7),
                            (pos[0] + np.sin(q1) * (L3 + L4 + L6) + L2)/(np.cos(q1) * (L5 + L7)))

        # other cases
        else:
            B1 = L5 - L7 * np.cos(q3)
            B2 = -L7 * np.sin(q3)
            q2 = np.arctan2((B1 * Z2 - B2 * Z1) / (B1 * B1 + B2 * B2),
                              (B1 * Z1 + B2 * Z2) / (B1 * B1 + B2 * B2))

        if q2 < constraints[2] or q2 > constraints[3]:
            raise NoSolutionException(pos, constraints)

        return np.array([q1, q2, q3])

    def __linearJacobian(self, Q):
        """
        Compute the Jacobian matrix for leg
        :param Q: Current configuration of a leg
        :return: The Jacobian matrix of a leg
        """
        q1, q2, q3 = Q
        L1, L2, L3, L4, L5, L6, L7 = self.L

        return np.matrix([[c(q1) * (-L3 - L4 - L6) - s(q1) * (c(q2) * L5 - L7 * c(q2 + q3)),
                           c(q1) * (L7 * s(q2 + q3) - s(q2) * L5), c(q1) * L7 * s(q2 + q3)],
                          [0, c(q2) * L5 - L7 * c(q2 + q3), -L7 * c(q2 + q3)],
                          [s(q1) * (-L3 - L4 - L6) + c(q1) * (c(q2) * L5 - L7 * c(q2 + q3)),
                           s(q1) * (L7 * s(q2 + q3) - s(q2) * L5), s(q1) * L7 * s(q2 + q3)]])

    def __T01(self, q1):
        """
        Homogeneous matrix from frame 0 to frame 1
        :param q1: first link of a leg
        :return: Homogeneous matrix
        """
        L = self.L
        return np.matrix([[c(q1), -s(q1), 0, -L[1]],
                          [0, 0, -1, -L[0]],
                          [s(q1), c(q1), 0, 0],
                          [0, 0, 0, 1]])

    def __T12(self, q2):
        """
        Homogeneous matrix from frame 1 to frame 2
        :param q2: second link of a leg
        :return: Homogeneous matrix
        """
        return np.matrix([[c(q2), -s(q2), 0, 0],
                          [0, 0, 1, 0],
                          [-s(q2), -c(q2), 0, 0],
                          [0, 0, 0, 1]])

    def __T23(self, q3):
        """
        Homogeneous matrix from frame 2 to frame 3
        :param q3: third link of a leg
        :return: Homogeneous matrix
        """
        L = self.L
        return np.matrix([[c(q3), -s(q3), 0, L[4]],
                          [s(q3), c(q3), 0, 0],
                          [0, 0, 1, L[2] + L[3]],
                          [0, 0, 0, 1]])

    def __T34(self):
        """
        Homogeneous matrix from frame 3 to frame 4
        :return: Homogeneous matrix
        """
        L = self.L
        return np.matrix([[1, 0, 0, -L[6]],
                          [0, 1, 0, 0],
                          [0, 0, 1, L[5]],
                          [0, 0, 0, 1]])

    def T0k(self, q, k, R):
        """
        Compute the all homogeneous matrix from 0 to k frame
        :param q: configuration of a leg
        :param k: number of frame
        :param R:
        :return:
        """
        funcs = [self.__T01, self.__T12, self.__T23, self.__T34]
        T0k = np.identity(4)
        for i in range(0, k):
            if i == 3:
                T0k = np.dot(T0k, funcs[i]())
            else:
                T0k = np.dot(T0k, funcs[i](q[i]))
        return R * T0k


def test_1():
    """
    Early test, all leg does the same movement, visualise the current configuration, move each links (q1, q2, q3)
    :return:
    """
    # imports
    from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory
    from solo_pybullet.model.robot.Viewer import Viewer

    # variables
    L = [0.1946, 0.0875, 0.014, 0.03745, 0.16, 0.008, 0.16]
    T = 1
    Lp = 0.15
    x0 = -L[1]
    y0 = -L[0] - Lp/2
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
        """
        Chose between the early foot trajectory(CycloidFootTrajectory) or the new one (BézierFootTrajectory)
        """
        # res[0:3, i] = CycloidFootTrajectory.f(t0, T, np.array([x0, y0, z0]), H, np.array([0, Lp]), dir=False)
        # res[3:6, i] = CycloidFootTrajectory.f(t0, T, np.array([x0, y0, z0]), H, np.array([0, Lp]), dir=False)
        # res[6:9, i] = CycloidFootTrajectory.f(t0, T, np.array([x0, y0, z0]), H, np.array([0, Lp]), dir=False)
        # res[9:12, i] = CycloidFootTrajectory.f(t0, T,np.array([x0, y0, z0]), H, np.array([0, Lp]), dir=False)
        res[0:3, i] = BezierFootTrajectory.f(t0, T, np.array([x0, y0, z0]), np.array([x0, y0 + Lp, z0 + H]))
        res[3:6, i] = BezierFootTrajectory.f(t0, T, np.array([x0, y0, z0]), np.array([x0, y0 + Lp, z0 + H]))
        res[6:9, i] = BezierFootTrajectory.f(t0, T, np.array([x0, y0, z0]), np.array([x0, y0 + Lp, z0 + H]))
        res[9:12, i] = BezierFootTrajectory.f(t0, T, np.array([x0, y0, z0]), np.array([x0, y0 + Lp, z0 + H]))

    Viewer.viewInverseKinematics(kinematics, res)


if __name__ == '__main__':
    test_1()
