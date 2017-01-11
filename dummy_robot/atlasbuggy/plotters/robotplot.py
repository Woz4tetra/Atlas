import mpl_toolkits.mplot3d.axes3d


class RobotPlot:
    def __init__(self, plot_name, plot_enabled=True, flat_plot=True,
                 x_range=None, y_range=None, z_range=None, range_offset=1,
                 line_segments=False, line_seg_freq=1,
                 **plot_properties):
        self.name = plot_name
        self.flat = flat_plot
        self.enabled = plot_enabled

        self.properties = plot_properties

        self.line_segments = line_segments
        self.line_seg_freq = line_seg_freq

        self.x_range = x_range
        if type(self.x_range) == tuple:
            self.x_range = list(x_range)
        self.y_range = y_range
        if type(self.y_range) == tuple:
            self.y_range = list(y_range)
        self.z_range = z_range
        if type(self.z_range) == tuple:
            self.z_range = list(z_range)
        self.range_offset = range_offset

        self.line_seg_counter = 0

        if self.line_segments:
            self.data = []
        else:
            self.data = [[] for _ in range(2 if flat_plot else 3)]

    def update(self, xs, ys, zs=None):
        if not self.enabled:
            return

        self.data[0] = xs
        min_x, max_x = min(xs), max(xs)
        self.x_range = self.update_range(min_x, self.x_range)
        self.x_range = self.update_range(max_x, self.x_range)

        self.data[1] = ys
        min_y, max_y = min(ys), max(ys)
        self.y_range = self.update_range(min_y, self.y_range)
        self.y_range = self.update_range(max_y, self.y_range)

        if not self.flat:
            self.data[2] = zs
            min_z, max_z = min(zs), max(zs)
            self.z_range = self.update_range(min_z, self.z_range)
            self.z_range = self.update_range(max_z, self.z_range)

    def append(self, x, y, z=None):
        if not self.enabled:
            return

        self.append_x(x)
        self.append_y(y)
        if not self.flat:
            self.append_z(z)

    def update_range(self, value, range_list):
        if range_list is None:
            range_list = [value - self.range_offset, value + self.range_offset]

        if value < range_list[0]:
            range_list[0] = value - self.range_offset
        if value > range_list[1]:
            range_list[1] = value + self.range_offset

        return range_list

    def append_x(self, x):
        self.data[0].append(x)
        self.x_range = self.update_range(x, self.x_range)

    def append_y(self, y):
        self.data[1].append(y)
        self.y_range = self.update_range(y, self.y_range)

    def append_z(self, z):
        self.data[2].append(z)
        self.z_range = self.update_range(z, self.z_range)


class RobotPlotCollection:
    def __init__(self, collection_name, *robot_plots, plot_enabled=True, flat_plot=True):
        self.name = collection_name
        self.plots = []

        self.flat = flat_plot
        self.enabled = plot_enabled
        for plot in robot_plots:
            if plot.enabled:
                plot.flat = self.flat
                plot.properties["label"] = plot.name
                self.plots.append(plot)

        if len(self.plots) == 0:
            self.enabled = False


    @property
    def x_range(self):
        x_min = min(self.plots, key=lambda plot: plot.x_range[0]).x_range[0]
        x_max = max(self.plots, key=lambda plot: plot.x_range[1]).x_range[1]
        return x_min, x_max

    @property
    def y_range(self):
        y_min = min(self.plots, key=lambda plot: plot.y_range[0]).y_range[0]
        y_max = max(self.plots, key=lambda plot: plot.y_range[1]).y_range[1]
        return y_min, y_max

    @property
    def z_range(self):
        z_min = min(self.plots, key=lambda plot: plot.z_range[0]).z_range[0]
        z_max = max(self.plots, key=lambda plot: plot.z_range[1]).z_range[1]
        return z_min, z_max
