import time
import traceback

from matplotlib import pyplot as plt

from atlasbuggy.plotters.baseplotter import BasePlotter
from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection


class LivePlotter(BasePlotter):
    def __init__(self, num_columns, *robot_plots, legend_args=None, lag_cap=0.005):
        if BasePlotter.fig_num > 0:
            raise Exception("Can't have multiple plotter instances!")

        super(LivePlotter, self).__init__(num_columns, legend_args, *robot_plots)

        for plot in self.robot_plots:
            if isinstance(plot, RobotPlot):
                if plot.flat:
                    self.lines[plot.name] = self.axes[plot.name].plot([], [], **plot.properties)[0]
                else:
                    self.lines[plot.name] = self.axes[plot.name].plot([], [], [], **plot.properties)[0]
            elif isinstance(plot, RobotPlotCollection):
                self.lines[plot.name] = {}

                if plot.flat:
                    for subplot in plot.plots:
                        self.lines[plot.name][subplot.name] = self.axes[plot.name].plot([], [], **subplot.properties)[0]
                else:
                    for subplot in plot.plots:
                        self.lines[plot.name][subplot.name] = \
                            self.axes[plot.name].plot([], [], [], **subplot.properties)[0]

        self.fig.canvas.mpl_connect('close_event', lambda event: self.handle_close())
        self.time0 = None
        self.lag_cap = lag_cap
        self.closed = False

        self.init_legend()
        plt.show(block=False)

    def should_update(self, packet_timestamp):
        # likely source of rare bug where the plot lags badly
        if self.time0 is None:
            self.time0 = time.time()
            return True

        current_time = time.time() - self.time0

        return abs(packet_timestamp - current_time) < self.lag_cap

    def plot(self):
        if self.closed or not self.plotter_enabled:
            return False

        for plot in self.robot_plots:
            if isinstance(plot, RobotPlot):
                self.lines[plot.name].set_xdata(plot.data[0])
                self.lines[plot.name].set_ydata(plot.data[1])
                if not plot.flat:
                    self.lines[plot.name].set_3d_properties(plot.data[2])

            elif isinstance(plot, RobotPlotCollection):
                for subplot in plot.plots:
                    self.lines[plot.name][subplot.name].set_xdata(subplot.data[0])
                    self.lines[plot.name][subplot.name].set_ydata(subplot.data[1])
                    if not plot.flat:
                        self.lines[plot.name][subplot.name].set_3d_properties(subplot.data[2])

            else:
                return False

            if plot.flat:
                self.axes[plot.name].set_xlim(plot.x_range)
                self.axes[plot.name].set_ylim(plot.y_range)
            else:
                self.axes[plot.name].set_xlim3d(plot.x_range)
                self.axes[plot.name].set_ylim3d(plot.y_range)
                self.axes[plot.name].set_zlim3d(plot.z_range)

        try:
            self.fig.canvas.draw()
            plt.pause(0.005)  # can't be less than ~0.005

        except BaseException as error:
            traceback.print_exc()
            print("plot closing:", error)

            self.close()
            return False

        return True

    def handle_close(self):
        self.close()

    def close(self):
        if self.plotter_enabled and not self.closed:
            self.closed = True
            plt.ioff()
            plt.gcf()
            plt.close('all')
