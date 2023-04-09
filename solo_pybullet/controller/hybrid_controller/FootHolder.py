import numpy as np


class FootHolder:
    def __init__(self, k, C, max_step_length, T):
        self.k = k
        self.C = C
        self.max_step_length = max_step_length
        self.T = T

    def get_feet_pos(self, Vx, Vy, omega, H):
        # get the module of speed
        V = np.linalg.norm(np.array([Vx, Vy]))

        # get the angle of the linear direction
        theta = np.arctan2(Vy, Vx)

        # compute the step length to reach the desired speed
        L = 0.5 * self.T * V

        if L > self.max_step_length:
            L = self.max_step_length

        # compute the start and end foot position in right foot frame
        Pi_FL = np.array([-self.C[0] + 0.5 * L * np.cos(theta + np.pi), self.C[1] + 0.5 * L * np.sin(theta + np.pi), 0, 1])
        Pf_FL = np.array([-self.C[0] + 0.5 * L * np.cos(theta), self.C[1] + 0.5 * L * np.sin(theta), H, 1])
        Pi_FR = np.array([self.C[0] + 0.5 * L * np.cos(theta + np.pi), self.C[1] + 0.5 * L * np.sin(theta + np.pi), 0, 1])
        Pf_FR = np.array([self.C[0] + 0.5 * L * np.cos(theta), self.C[1] + 0.5 * L * np.sin(theta), H, 1])
        Pi_HL = np.array([-self.C[0] + 0.5 * L * np.cos(theta + np.pi), -self.C[1] + 0.5 * L * np.sin(theta + np.pi), 0, 1])
        Pf_HL = np.array([-self.C[0] + 0.5 * L * np.cos(theta), -self.C[1] + 0.5 * L * np.sin(theta), H, 1])
        Pi_HR = np.array([self.C[0] + 0.5 * L * np.cos(theta + np.pi), -self.C[1] + 0.5 * L * np.sin(theta + np.pi), 0, 1])
        Pf_HR = np.array([self.C[0] + 0.5 * L * np.cos(theta), -self.C[1] + 0.5 * L * np.sin(theta), H, 1])

        Pi_FL = Pi_FL @ self.k.kinematics.r['FL']
        Pf_FL = Pf_FL @ self.k.kinematics.r['FL']
        Pi_FR = Pi_FR @ self.k.kinematics.r['FR']
        Pf_FR = Pf_FR @ self.k.kinematics.r['FR']
        Pi_HL = Pi_HL @ self.k.kinematics.r['HL']
        Pf_HL = Pf_HL @ self.k.kinematics.r['HL']
        Pi_HR = Pi_HR @ self.k.kinematics.r['HR']
        Pf_HR = Pf_HR @ self.k.kinematics.r['HR']

        return np.vstack((Pi_FL[0, 0:3], Pi_FR[0, 0:3], Pi_HL[0, 0:3], Pi_HR[0, 0:3])).A1, np.vstack((Pf_FL[0, 0:3], Pf_FR[0, 0:3], Pf_HL[0, 0:3], Pf_HR[0, 0:3])).A1
