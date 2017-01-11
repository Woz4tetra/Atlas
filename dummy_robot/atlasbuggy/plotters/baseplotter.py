from matplotlib import pyplot as plt


class BasePlotter:
    fig_num = 0

    def __init__(self, num_columns, legend_args, *robot_plots):
        self.closed = True

        self.robot_plots = []
        for plot in robot_plots:
            if plot.enabled:
                self.robot_plots.append(plot)

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

        self.closed = False

    def init_legend(self):
        if self.legend_args is None:
            self.legend_args = {}

        if "fontsize" not in self.legend_args:
            self.legend_args["fontsize"] = 'x-small'
        if "shadow" not in self.legend_args:
            self.legend_args["shadow"] = 'True'

        plt.legend(**self.legend_args)

    def plot(self):
        return True
