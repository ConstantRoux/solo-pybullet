import matplotlib.pyplot as plt


class Logger:
    def __init__(self):
        self.key = 0
        self.data = {}

    def create_channel(self, title, data):
        self.data[self.key] = (title, data)
        self.key += 1

    def plot_channel(self, indices, start_index, end_index, nrows, ncolumns):
        linestyles = ['solid', 'dashed', 'dotted']
        f, ax = plt.subplots(nrows, ncolumns)
        for i in range(nrows):
            for k in range(ncolumns):
                for l_i, j in enumerate(indices):
                    ax[i, k].plot(self.data[j][1][nrows * k + i, start_index:end_index], linestyle=linestyles[l_i % 3])
        plt.figlegend([self.data[i][0] for i in indices])
        f.tight_layout()
        plt.show()
