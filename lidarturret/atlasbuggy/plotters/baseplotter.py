"""
The base class shared by liveplotter and staticplotter. Contains properties shared by the two types.
"""

from matplotlib import pyplot as plt


class BasePlotter:
    fig_num = 0

    def __init__(self, num_columns, legend_args, *robot_plots):
        """
        A plotter is one matplotlib figure. Having multiple robot plots creates subplots
        :param num_columns: Configure how the subplots are arranged
        :param legend_args: dictionary of arguments to pass to plt.legend
        :param robot_plots: RobotPlot or RobotPlotCollection instances. Each one will be a subplot
        """
        self.robot_plots = []
        for plot in robot_plots:
            if plot.enabled:
                self.robot_plots.append(plot)

        if len(self.robot_plots) == 0:
            self.plotter_enabled = False
            return
        else:
            self.plotter_enabled = True

        num_plots = len(self.robot_plots)
        if num_plots < num_columns:
            num_columns = num_plots

        num_rows = num_plots // num_columns
        num_rows += num_plots % num_columns

        self.legend_args = legend_args

        self.axes = {}
        self.lines = {}

        self.fig = plt.figure(BasePlotter.fig_num)
        BasePlotter.fig_num += 1

        plot_num = 1
        for plot in self.robot_plots:
            if plot.flat:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num)
            else:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num, projection='3d')

            self.axes[plot.name].set_title(plot.name)
            plot_num += 1

    def init_legend(self):
        """
        Create a legend. Static and live plots need them created at different times
        :return:
        """
        if self.legend_args is None:
            self.legend_args = {}

        if "fontsize" not in self.legend_args:
            self.legend_args["fontsize"] = 'x-small'
        if "shadow" not in self.legend_args:
            self.legend_args["shadow"] = 'True'

        plt.legend(**self.legend_args)

    def plot(self):
        """
        Header method. Implemented in static and live plotter
        :return: True or False depending on if the program should exit or not
        """
        return True

