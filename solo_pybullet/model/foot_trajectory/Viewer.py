from matplotlib import pyplot as plt


class Viewer:
    @staticmethod
    def viewFootTrajectory(P, dP):
        # TODO better display
        f, ax = plt.subplots()
        f.suptitle(r'Visualisation de la trajectoire dans l\'espace cartésien')
        ax = plt.axes(projection='3d')
        ax.set_proj_type('ortho')
        ax.scatter3D(P[0, :], P[1, :], P[2, :])
        plt.plot()

        f, ax = plt.subplots(2, 3)
        for i in range(3):
            ax[0, i].scatter(P[i, :])
            ax[1, i].scatter(dP[i, :])
        plt.plot()
