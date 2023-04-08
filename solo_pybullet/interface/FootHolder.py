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
        L = self.T * V

        if L > self.max_step_length:
            L = self.max_step_length

        # compute the start and end foot position in right foot frame
        Pi = np.array([self.C[0] + 0.5 * L * np.cos(theta + np.pi), self.C[1] + 0.5 * L * np.sin(theta + np.pi), 0, 1])
        Pf = np.array([self.C[0] + 0.5 * L * np.cos(theta), self.C[1] + 0.5 * L * np.sin(theta), H, 1])

        return Pi[0:3], Pf[0:3]

