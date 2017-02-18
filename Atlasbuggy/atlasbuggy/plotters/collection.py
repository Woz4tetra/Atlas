

class RobotPlotCollection:
    def __init__(self, collection_name, *robot_plots, plot_enabled=True, flat_plot=True):
        """
        If you want to plot multiple lines on one subplot, pass your robot plot objects to a collection.
        Each robot plot will be treated as one line on the subplot.

        :param collection_name: plot title and internal reference name. Make sure its unique
        :param robot_plots: RobotPlot instances
        :param plot_enabled: Turn collections on and off easily with this flag. If False, all subplots will be disabled
        :param flat_plot: 2D if True, 3D if False. All subplots will be assigned this value
        """
        self.name = collection_name
        self.plots = []

        # unify key properties between all subplots
        self.flat = flat_plot
        self.enabled = plot_enabled
        for plot in robot_plots:
            if plot.enabled:
                plot.flat = self.flat
                plot.properties["label"] = plot.name
                self.plots.append(plot)
                plot.collection_plot = self

        if len(self.plots) == 0:
            self.enabled = False

    def ranges_set(self):
        for plot in self.plots:
            if not plot.ranges_set():
                return False
        return True

    def get_range(self, axis_num):
        ranges = []
        for plot in self.plots:
            if plot.ranges[axis_num] is not None:
                ranges.append(plot.ranges[axis_num])

        if len(ranges) == 0:
            return -0.001, 0.001
        else:
            return min(ranges, key=lambda range: range[0])[0], \
                   max(ranges, key=lambda range: range[1])[1]

    @property
    def x_range(self):
        return self.get_range(0)

    @property
    def y_range(self):
        return self.get_range(1)

    @property
    def z_range(self):
        return self.get_range(2)