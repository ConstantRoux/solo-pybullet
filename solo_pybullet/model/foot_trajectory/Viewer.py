from matplotlib import pyplot as plt


class Viewer:
    @staticmethod
    def viewFootTrajectory(points):
        f, ax = plt.subplots()
        f.suptitle(r'Visualisation de la trajectoire')
        ax = plt.axes(projection='3d')
        ax.set_proj_type('ortho')
        ax.scatter3D(points[0, :], points[1, :], points[2, :])
