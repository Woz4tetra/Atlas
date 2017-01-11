from matplotlib import pyplot as plt
from atlasbuggy.plotters.baseplotter import BasePlotter


class StaticPlotter(BasePlotter):
    def __init__(self, num_columns, *robot_plots):
        super(StaticPlotter, self).__init__(num_columns, *robot_plots)

    def plot(self):
        for plot in self.robot_plots:
            if plot.flat:
                self.lines[plot.name] = self.axes[plot.name].plot(
                    plot.data[0], plot.data[1], **plot.properties)[0]
            else:
                self.lines[plot.name] = self.axes[plot.name].plot(
                    plot.data[0], plot.data[1], plot.data[2], **plot.properties)[0]

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')
        plt.show()
