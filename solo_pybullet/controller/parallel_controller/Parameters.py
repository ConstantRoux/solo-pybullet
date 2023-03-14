import pybullet as p


class Parameters:
    @staticmethod
    def init_params():
        global T, xF0, yF0, z0, dx, dz
        p.addUserDebugParameter("  tx  ", -0.1, 0.1, 0)
        p.addUserDebugParameter("  ty  ", -0.1, 0.1, 0)
        p.addUserDebugParameter("  tz  ", -0.3, -0.05, -0.2)
        p.addUserDebugParameter("  rx  ", -0.5, 0.5, 0)
        p.addUserDebugParameter("  ry  ", -0.5, 0.5, 0)
        p.addUserDebugParameter("  rz  ", -0.5, 0.5, 0)

    @staticmethod
    def get_params():
        return p.readUserDebugParameter(0), p.readUserDebugParameter(1), p.readUserDebugParameter(2), p.readUserDebugParameter(3), p.readUserDebugParameter(4), p.readUserDebugParameter(5)