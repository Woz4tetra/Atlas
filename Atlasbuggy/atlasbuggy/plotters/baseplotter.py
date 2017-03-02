"""
The base class shared by liveplotter and staticplotter. Contains properties shared by the two types.
"""

from matplotlib import pyplot as plt
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection


class BasePlotter:
    fig_num = 0

    def __init__(self, num_columns, legend_args, enable_legend, matplotlib_events, *robot_plots):
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
        else:
            self.plotter_enabled = True

        self.legend_args = legend_args
        self.enable_legend = enable_legend

        self.axes = {}
        self.lines = {}
        self.plots_dict = {}
        self.extra_elements = {}

        if self.plotter_enabled:
            self.fig = plt.figure(BasePlotter.fig_num)
            if matplotlib_events is not None:
                for event_name in matplotlib_events:
                    self.fig.canvas.mpl_connect(event_name, matplotlib_events[event_name])
            BasePlotter.fig_num += 1

            num_plots = len(self.robot_plots)
            if num_plots < num_columns:
                num_columns = num_plots

            num_rows = num_plots // num_columns
            num_rows += num_plots % num_columns

            plot_num = 1
            for plot in self.robot_plots:
                self.plots_dict[plot.name] = plot
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
        if self.enable_legend:
            if self.legend_args is None:
                self.legend_args = {}

            if "fontsize" not in self.legend_args:
                self.legend_args["fontsize"] = 'x-small'
            if "shadow" not in self.legend_args:
                self.legend_args["shadow"] = 'True'

            plt.legend(**self.legend_args)

    def get_name(self, arg):
        if type(arg) == str:
            return arg
        elif isinstance(arg, RobotPlot) or isinstance(arg, RobotPlotCollection):
            return arg.name
        else:
            return None

    def get_axis(self, arg):
        if self.plotter_enabled:
            plot_name = self.get_name(arg)

            if plot_name in self.axes.keys():
                return self.axes[plot_name]
        return None

    def draw_dot(self, arg, x, y, z=None, **dot_properties):
        if self.plotter_enabled:
            plot_name = self.get_name(arg)
            if plot_name is None:
                return

            if plot_name in self.axes.keys():
                if self.plots_dict[plot_name].flat:
                    self.axes[plot_name].plot(x, y, 'o', **dot_properties)
                else:
                    self.axes[plot_name].plot([x], [y], [z], 'o', **dot_properties)

    def draw_text(self, arg, text, x, y, z=None, text_name=None, **text_properties):
        if self.plotter_enabled:
            plot_name = self.get_name(arg)
            if plot_name is None:
                return

            if plot_name in self.axes.keys():
                if text_name is not None and text_name in self.extra_elements:
                    self.extra_elements[text_name].remove()
                if self.plots_dict[plot_name].flat:
                    self.extra_elements[text_name] = self.axes[plot_name].text(x, y, text, **text_properties)
                else:
                    self.extra_elements[text_name] = self.axes[plot_name].text([x], [y], [z], text, **text_properties)

    def set_line_props(self, arg, **kwargs):
        if self.plotter_enabled:
            plot_name = self.get_name(arg)
            if plot_name is None:
                return

            if isinstance(arg, RobotPlot) and arg.collection_plot is not None:
                collection_name = arg.collection_plot.name
                self.lines[collection_name][plot_name].set(**kwargs)
            else:
                self.lines[plot_name].set(**kwargs)

    def plot(self):
        """
        Header method. Implemented in static and live plotter
        :return: True or False depending on if the program should exit or not
        """
        return True
