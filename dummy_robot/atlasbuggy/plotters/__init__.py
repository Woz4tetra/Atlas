import mpl_toolkits.mplot3d.axes3d as p3


class RobotPlot:
    def __init__(self, plot_name, plot_enabled=True, flat_plot=True,
                 x_range=None, y_range=None, z_range=None, range_offset=1,
                 line_segments=False, line_seg_freq=1, skip_count=0,
                 **plot_properties):
        self.name = plot_name
        self.flat = flat_plot
        self.enabled = plot_enabled

        self.properties = plot_properties

        self.line_segments = line_segments
        self.line_seg_freq = line_seg_freq
        self.skip_count = skip_count
        self.skip_counter = 0

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

    def should_skip(self):
        if self.skip_count == 0:
            return False
        else:
            self.skip_counter += 1
            return self.skip_counter % self.skip_count

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
