import numpy as np
from matplotlib import pyplot as plt
from matplotlib.widgets import Slider


class Viewer:
    @staticmethod
    def viewConfigurations(kinematics):
        pass

    @staticmethod
    def viewForwardKinematics(kinematics):
        f, ax = plt.subplots()
        f.suptitle(r'Forward geometric model view')
        ax_q = [None] * 3
        sl_q = [None] * 3
        ax = plt.axes(projection='3d')
        ax.set_proj_type('ortho')
        f.subplots_adjust(left=0.30)
        legends = ['FL', 'FR', 'HL', 'HR']

        q = np.zeros((3,))

        def draw():
            c_r = np.zeros((5, 3, 4))
            for i in range(5):
                for j in range(4):
                    if i == 0:
                        c_r[i, :, j] = kinematics.r[kinematics.labels[j]][:-1, 3].T
                    else:
                        c_r[i, :, j] = np.transpose(kinematics.T0k(q, i, kinematics.r[kinematics.labels[j]])[:-1, 3])

            ax.set_aspect('equal')
            ax.axes.set_xlim3d(left=-1, right=1)
            ax.axes.set_ylim3d(bottom=-1, top=1)
            ax.axes.set_zlim3d(bottom=-1, top=1)
            for j in range(4):
                ax.plot3D(c_r[:, 0, j], c_r[:, 1, j], c_r[:, 2, j], linewidth='1', label=legends[j])

            ax.set_xlabel(r'$x$')
            ax.set_ylabel(r'$y$')
            ax.set_zlabel(r'$z$')
            ax.legend()

        draw()

        def update(value):
            for i in range(q.shape[0]):
                q[i] = sl_q[i].val
            ax.clear()
            draw()

        # sliders
        for i in range(3):
            ax_q[i] = f.add_axes([0.05 + 0.05 * i, 0.1, 0.0225, 0.8])
            sl_q[i] = Slider(
                ax=ax_q[i],
                label='q' + str(i + 1),
                valmin=-np.pi,
                valmax=np.pi,
                valinit=0,
                orientation="vertical")
            sl_q[i].on_changed(update)

        plt.show()

    @staticmethod
    def viewInverseKinematics(kinematics, points):
        Q = np.empty((12, points.shape[1]))
        pos = np.zeros((5 * points.shape[1], 3, 4))
        constraints = np.array([0, np.pi, -np.pi, np.pi, 0, np.pi] * 4)

        for i in range(points.shape[1]):
            Q[:, i], _ = kinematics.inverse_kinematics(points[:, i], np.zeros((12,)), constraints)
            for j in range(1, 5):
                for k in range(4):
                    pos[5 * i + j, :, k] = np.transpose(kinematics.T0k(Q[3 * k:3 * (k + 1), i], j, kinematics.r[kinematics.labels[k]])[:-1, 3])

        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title('Inverse Geometric model view')
        ax.plot3D(points[0, :], points[1, :], points[2, :], linestyle='dashed', label='expected foot trajectory')
        for k in range(4):
            for i in range(points.shape[1]):
                ax.plot3D(pos[5 * i: 5 * (i + 1), 0, k], pos[5 * i: 5 * (i + 1), 1, k], pos[5 * i: 5 * (i + 1), 2, k], alpha=0.5)
        ax.set_xlabel(r'$x$')
        ax.set_ylabel(r'$y$')
        ax.set_zlabel(r'$z$')
        ax.legend()
        plt.show()