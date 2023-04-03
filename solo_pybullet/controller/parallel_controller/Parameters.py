import pybullet as p


class Parameters:
    @staticmethod
    def init_params():
        """
        sliders for the test in pybullet
        :return:
        """
        global T, xF0, yF0, z0, dx, dz
        p.addUserDebugParameter("  tx  ", -0.25, 0.25, 0)
        p.addUserDebugParameter("  ty  ", -0.25, 0.25, 0)
        p.addUserDebugParameter("  tz  ", 0., 0.32, 0.16)
        p.addUserDebugParameter("  rx  ", -1, 1, 0)
        p.addUserDebugParameter("  ry  ", -1.5, 1.5, 0)
        p.addUserDebugParameter("  rz  ", -0.8, 0.8, 0)

    @staticmethod
    def get_params():
        return p.readUserDebugParameter(0), p.readUserDebugParameter(1), p.readUserDebugParameter(2), p.readUserDebugParameter(3), p.readUserDebugParameter(4), p.readUserDebugParameter(5)