import time
import traceback

from matplotlib import pyplot as plt

from atlasbuggy.plotters.baseplotter import BasePlotter

class LivePlotter(BasePlotter):
    def __init__(self, num_columns, *robot_plots):
        super(LivePlotter, self).__init__(num_columns, *robot_plots)

        self.time0 = None

        plt.show(block=False)

    def should_update(self, packet_timestamp):
        if self.time0 is None:
            self.time0 = time.time()
            return True

        current_time = time.time() - self.time0
        return abs(packet_timestamp - current_time) < 0.01

    def plot(self):
        if self.closed:
            return False

        for plot in self.robot_plots:
            if plot.should_skip():
                self.should_skip = True
                continue

            self.lines[plot.name].set_xdata(plot.data[0])
            self.lines[plot.name].set_ydata(plot.data[1])

            if not plot.flat:
                self.lines[plot.name].set_3d_properties(plot.data[2])

            if plot.flat:
                self.axes[plot.name].set_xlim(plot.x_range[0], plot.x_range[1])
                self.axes[plot.name].set_ylim(plot.y_range[0], plot.y_range[1])
            else:
                self.axes[plot.name].set_xlim3d([plot.x_range[0], plot.x_range[1]])
                self.axes[plot.name].set_ylim3d([plot.y_range[0], plot.y_range[1]])
                self.axes[plot.name].set_zlim3d([plot.z_range[0], plot.z_range[1]])

        if self.should_skip:
            self.should_skip = False
            return True

        try:
            self.fig.canvas.draw()
            plt.pause(0.005)  # can't be less than ~0.005

        except BaseException as error:
            traceback.print_exc()
            print("plot closing:", error)

            self.close()
            return False

        return True
