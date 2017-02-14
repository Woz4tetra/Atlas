"""
Contains the LivePlotter class, a subclass of BasePlotter. This class plots incoming data in real time
according to properties defined in RobotPlot.
"""

import time
import traceback

from matplotlib import pyplot as plt

from atlasbuggy.plotters.baseplotter import BasePlotter
from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection


class LivePlotter(BasePlotter):
    initialized = False

    def __init__(self, num_columns, *robot_plots, legend_args=None, lag_cap=0.005, plot_skip_count=0):
        """
        Only one LivePlotter instance can run at one time. Multiple interactive matplotlib
        windows don't behave well. This also conserves CPU usage.

        :param num_columns: Configure how the subplots are arranged
        :param robot_plots: RobotPlot or RobotPlotCollection instances. Each one will be a subplot
        :param legend_args: dictionary of arguments to pass to plt.legend
        :param lag_cap: Constrains how out of sync the plot can be with incoming packets. If the plot
            is causing a time difference greater than the one specified, skip plotting the incoming data
            until the plotter comes back in sync.
        """
        if LivePlotter.initialized:
            raise Exception("Can't have multiple plotter instances!")
        LivePlotter.initialized = True

        super(LivePlotter, self).__init__(num_columns, legend_args, *robot_plots)

        # create a plot line for each RobotPlot or RobotPlotCollection.
        for plot in self.robot_plots:
            if isinstance(plot, RobotPlot):
                if plot.flat:  # if the plot is 2D
                    self.lines[plot.name] = self.axes[plot.name].plot([], [], **plot.properties)[0]
                else:
                    self.lines[plot.name] = self.axes[plot.name].plot([], [], [], **plot.properties)[0]
            elif isinstance(plot, RobotPlotCollection):  # similar to RobotPlot initialization except there are subplots
                self.lines[plot.name] = {}

                if plot.flat:
                    for subplot in plot.plots:
                        self.lines[plot.name][subplot.name] = self.axes[plot.name].plot([], [], **subplot.properties)[0]
                else:
                    for subplot in plot.plots:
                        self.lines[plot.name][subplot.name] = \
                            self.axes[plot.name].plot([], [], [], **subplot.properties)[0]

        # define a clean close event
        self.fig.canvas.mpl_connect('close_event', lambda event: self.close())
        self.time0 = None
        self.lag_cap = lag_cap
        self.plot_skip_count = plot_skip_count
        self.skip_counter = 0
        self.closed = False

        self.init_legend()
        plt.show(block=False)

    def start_time(self, time0):
        """
        Supply a start time. This keeps all timers in sync

        :param time0: a value supplied by time.time()
        :return: None
        """
        self.time0 = time0

    def should_update(self, packet_timestamp=None):
        """
        Signal whether or not the plot should update using self.lag_cap.
        If the packet_timestamp - current_time is greater than self.lag_cap, return False

        :param packet_timestamp: A timestamp received with a packet
        :return: True or False whether the plot should update or not
        """
        # likely source of rare bug where the plot lags badly
        if self.time0 is not None:
            current_time = time.time() - self.time0

            lag_status = abs(packet_timestamp - current_time) < self.lag_cap
        else:
            lag_status = True

        self.skip_counter += 1
        skip_status = self.plot_skip_count > 0 and self.skip_counter % self.plot_skip_count == 0
        return lag_status and skip_status

    def plot(self):
        """
        Update plot using data supplied to the robot plot objects
        :return: True or False if the plotting operation was successful
        """
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
                # print(plot.x_range, end=", ")
                # print(plot.y_range)
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

    def freeze_plot(self):
        plt.ioff()
        plt.gcf()
        plt.show()

    def close(self):
        """
        Close the plot safely
        :return: None
        """
        if self.plotter_enabled and not self.closed:
            self.closed = True
            plt.ioff()
            plt.gcf()
            plt.close('all')
