from solo_pybullet.model.com_trajectory.BezierComTrajectory import BezierComTrajectory
import numpy as np
from scipy.optimize import minimize, brute, fmin

from solo_pybullet.model.foot_trajectory.CycloidFootTrajectory import CycloidFootTrajectory


class OptimizationController:
    def __init__(self, kinematics, T, dt, H, Lp, z0, Pc):
        self.in_gait = False
        self.in_t = 0
        self.T = T
        self.dt = dt
        self.H = H
        self.Lp = Lp
        self.z0 = z0
        self.Pc = Pc
        self.k = kinematics
        self.des_Pf = np.zeros((8,))
        self.Pf = np.zeros((8,))

    @staticmethod
    def minimize_normal_crossing(s, Pd, Pc):
        xd, yd = Pd
        xs, ys = BezierComTrajectory.c(s, Pc)
        tx, ty = BezierComTrajectory.tc(s, Pc)
        if ty == 0:
            return (np.abs(ys - yd) + 1) * np.linalg.norm(Pd - BezierComTrajectory.c(s, Pc))
        return (np.abs(-(tx / ty) * (xd - xs) + ys - yd) + 1) * np.linalg.norm(Pd - BezierComTrajectory.c(s, Pc))

    @staticmethod
    def minimize_segment(ds, sk, Pc, Lp):
        return np.abs(np.linalg.norm(BezierComTrajectory.c(sk, Pc) - BezierComTrajectory.c(sk + ds[0], Pc)) - Lp)

    @staticmethod
    def minimize_curve_error(x, Pb, Pd):
        # update Pb with the new feet's positions
        Rk = np.array([[np.cos(Pb[2]), -np.sin(Pb[2])], [np.sin(Pb[2]), np.cos(Pb[2])]])
        for i in range(4):
            x[2 * i:2 * (i + 1)] = Rk @ x[2 * i:2 * (i + 1)]

        # compute euclidian distance between updated Pb and Pd
        dist = np.linalg.norm(Pd - (Pb + np.array([np.mean(x[::2]), np.mean(x[1::2]), Pb[2]])))

        # return objective value
        return dist

    @staticmethod
    def path_following(Pb, Pc, L1, L2, Lp):
        ###################################
        #         INITIALISATION          #
        ###################################

        # optimisation parameters
        x_init = np.array([L1, L2, L1, -L2, -L1, L2, -L1, -L2])
        cons = ({'type': 'ineq', 'fun': lambda x: -((x[0] - L1) ** 2 + (x[1] - L2) ** 2 - (Lp / 2) ** 2)},  # FL
                {'type': 'ineq', 'fun': lambda x: -((x[2] - L1) ** 2 + (x[3] + L2) ** 2 - (Lp / 2) ** 2)},  # FR
                {'type': 'ineq', 'fun': lambda x: -((x[4] + L1) ** 2 + (x[5] - L2) ** 2 - (Lp / 2) ** 2)},  # HL
                {'type': 'ineq', 'fun': lambda x: -((x[6] + L1) ** 2 + (x[7] + L2) ** 2 - (Lp / 2) ** 2)},  # HR
                {'type': 'ineq', 'fun': lambda x: np.abs(x[1] - x[3]) - L2},
                {'type': 'ineq', 'fun': lambda x: np.abs(x[5] - x[7]) - L2},
                {'type': 'ineq', 'fun': lambda x: np.abs(x[0] - x[4]) - L1},
                {'type': 'ineq', 'fun': lambda x: np.abs(x[2] - x[6]) - L1})

        ###################################
        #           OPTIMIZATION          #
        ###################################
        # get the s-value where the normal of the reference curve crossing the robot
        sk, *_ = brute(OptimizationController.minimize_normal_crossing, ((0, 1),), args=(Pb[0:2], Pc), finish=fmin)

        # get the position of the desired position on the curve with the constraint that the segment between the curve
        # at sk and sk+delta has to be equal to Lp
        dk = minimize(OptimizationController.minimize_segment, x0=np.array([sk]), args=(sk, Pc, Lp), method='SLSQP',
                      bounds=((0, None),), tol=1e-6).x
        sk = sk + dk if sk + dk < 1 else 1
        T = BezierComTrajectory.tc(sk, Pc)
        Pd = np.append(BezierComTrajectory.c(sk, Pc), np.arctan2(T[1], T[0]))

        # get the new feet's positions
        x = minimize(OptimizationController.minimize_curve_error, x0=x_init, args=(Pb, Pd), method='SLSQP',
                     constraints=cons, tol=1e-6).x

        return x

    def controller(self, t, constraints, Pb, Pf):
        if not self.in_gait:
            # get current feet's positions
            self.Pf = Pf

            # get best relative feet's positions
            self.des_Pf = OptimizationController.path_following(Pb, self.Pc, self.k.L[0], self.k.L[1], self.Lp)

            # converts des_Pf in each foot frame (MAYBE)
            for i in range(4):
                self.des_Pf[2 * i], self.des_Pf[2 * i + 1] = -self.des_Pf[2 * i + 1], self.des_Pf[2 * i]

            # pass in gait mode
            self.in_gait = True

        if self.in_gait:
            # generate feet's trajectories (f and df) between Pf and new Pf
            P = np.zeros((12,))
            P[0:3] = CycloidFootTrajectory.f(t + self.T / 2, self.T, np.append(self.Pf[0:2], self.z0), self.H, (self.des_Pf[0:2] - self.Pf[0:2]), dir=False)
            P[3:6] = CycloidFootTrajectory.f(t, self.T, np.append(self.Pf[2:4], self.z0), self.H, (self.des_Pf[2:4] - self.Pf[2:4]), dir=False)
            P[6:9] = CycloidFootTrajectory.f((t + self.T / 2), self.T, np.append(self.Pf[4:6], self.z0), self.H, (self.des_Pf[4:6] - self.Pf[4:6]), dir=True)
            P[9:12] = CycloidFootTrajectory.f((t + self.T / 2) + self.T / 2, self.T, np.append(self.Pf[6:8], self.z0), self.H, (self.des_Pf[6:8] - self.Pf[6:8]), dir=True)

            dP = np.zeros((12,))
            dP[0:3] = CycloidFootTrajectory.df(t + self.T / 2, self.T, self.H, (self.des_Pf[0:2] - self.Pf[0:2]), dir=False)
            dP[3:6] = CycloidFootTrajectory.df(t, self.T, self.H, (self.des_Pf[2:4] - self.Pf[2:4]), dir=False)
            dP[6:9] = CycloidFootTrajectory.df((t + self.T / 2), self.T, self.H, (self.des_Pf[4:6] - self.Pf[4:6]), dir=True)
            dP[9:12] = CycloidFootTrajectory.df((t + self.T / 2) + self.T / 2, self.T, self.H, (self.des_Pf[6:8] - self.Pf[6:8]), dir=True)

            # update intern timer
            self.in_t += self.dt

            # if one period has passed, reset intern timer and switch gait mode
            if self.in_t > self.T:
                self.in_t = 0
                self.in_gait = False

            # return q, dq
            return self.k.inverse_kinematics(P, dP, constraints)
