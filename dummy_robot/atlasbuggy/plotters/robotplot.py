import mpl_toolkits.mplot3d.axes3d


class RobotPlot:
    def __init__(self, plot_name, plot_enabled=True, flat_plot=True,
                 x_range=None, y_range=None, z_range=None, range_offset=0,
                 x_lim=None, y_lim=None, z_lim=None, skip_count=0,
                 max_length=None, **plot_properties):
        self.name = plot_name
        self.flat = flat_plot
        self.enabled = plot_enabled
        self.max_length = max_length
        self.skip_count = skip_count
        self.skip_counter = 0

        self.properties = plot_properties

        if type(x_range) == tuple:
            x_range = list(x_range)
        if type(y_range) == tuple:
            y_range = list(y_range)
        if type(z_range) == tuple:
            z_range = list(z_range)

        self.ranges = [x_range, y_range, z_range]
        self.ranges_contrained = []
        for r in self.ranges:
            self.ranges_contrained.append(False if r is None else True)
        self.range_offset = range_offset

        if type(x_lim) == tuple:
            x_lim = list(x_lim)
        if type(y_lim) == tuple:
            y_lim = list(y_lim)
        if type(z_lim) == tuple:
            z_lim = list(z_lim)

        self.limits = [x_lim, y_lim, z_lim]

        self.line_seg_counter = 0

        # if self.line_segments:
        #     self.data = []
        # else:
        self.data = [[] for _ in range(2 if flat_plot else 3)]

    def update(self, xs, ys, zs=None):
        if not self.enabled:
            return

        self.data[0] = xs
        min_x, max_x = min(xs), max(xs)
        self._update_range(min_x, 0)
        self._update_range(max_x, 0)

        self.data[1] = ys
        min_y, max_y = min(ys), max(ys)
        self._update_range(min_y, 1)
        self._update_range(max_y, 1)

        if not self.flat:
            self.data[2] = zs
            min_z, max_z = min(zs), max(zs)
            self._update_range(min_z, 2)
            self._update_range(max_z, 2)

    def append(self, x, y, z=None):
        if not self.enabled:
            return

        self.skip_counter += 1
        if self.skip_count > 0 and self.skip_counter % self.skip_count != 0:
            return

        self._append_x(x)
        self._append_y(y)
        if not self.flat:
            self._append_z(z)

    def _update_range(self, value, axis_num):
        if self.ranges[axis_num] is None:
            if self.range_offset == 0:
                offset = 0.001
            else:
                offset = self.range_offset
            self.ranges[axis_num] = [value - offset, value + offset]

        if value < self.ranges[axis_num][0]:
            self.ranges[axis_num][0] = value - self.range_offset
        if value > self.ranges[axis_num][1]:
            self.ranges[axis_num][1] = value + self.range_offset

        if self.limits[axis_num] is not None:
            if self.ranges[axis_num][0] > self.limits[axis_num][0]:
                self.ranges[axis_num][0] = self.limits[axis_num][0]
            if self.ranges[axis_num][1] > self.limits[axis_num][1]:
                self.ranges[axis_num][1] = self.limits[axis_num][1]

    def _append_to_axis(self, datum, axis_num):
        self.data[axis_num].append(datum)
        self._update_range(datum, axis_num)

        if self.max_length is not None and len(self.data[axis_num]) > self.max_length:
            self.data[axis_num].pop(0)
            if not self.ranges_contrained[axis_num]:
                self.ranges[axis_num][0] = min(self.data[axis_num])
                self.ranges[axis_num][1] = max(self.data[axis_num])

    def _append_x(self, x):
        self._append_to_axis(x, 0)

    def _append_y(self, y):
        self._append_to_axis(y, 1)

    def _append_z(self, z):
        self._append_to_axis(z, 2)

    @property
    def x_range(self):
        return self.ranges[0]

    @property
    def y_range(self):
        return self.ranges[1]

    @property
    def z_range(self):
        return self.ranges[2]


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
