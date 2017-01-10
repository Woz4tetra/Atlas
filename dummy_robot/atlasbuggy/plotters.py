import math
import traceback
from collections import defaultdict

import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import pyplot as plt
from atlasbuggy.logs import *

pickled_sim_directory = ":simulations"


class RobotPlot:
    def __init__(self, plot_name, flat_plot=True, x_range=None, y_range=None, z_range=None,
                 line_segments=False, log_based_plot=True, line_seg_freq=1, skip_count=0,
                 plot_enabled=True, range_offset=1,
                 **plot_properties):
        self.name = plot_name
        self.flat = flat_plot
        self.enabled = plot_enabled

        self.properties = plot_properties

        self.line_segments = line_segments
        self.log_based_plot = log_based_plot
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


class LivePlotter:
    initialized = False

    def __init__(self, num_columns, *robot_plots):
        if LivePlotter.initialized:
            raise Exception("Can't have multiple LivePlotter instances!")
        LivePlotter.initialized = True

        self.should_skip = False

        self.closed = True

        self.robot_plots = list(robot_plots)
        for plot in self.robot_plots:
            if not plot.enabled:
                self.robot_plots.remove(plot)
                continue

        num_plots = len(self.robot_plots)
        num_rows = num_plots // num_columns
        num_rows += num_plots % num_columns

        self.axes = {}
        self.live_lines = {}

        self.fig = plt.figure(0)
        self.fig.canvas.mpl_connect('close_event', lambda event: self.handle_close())

        plot_num = 1
        for plot in self.robot_plots:
            if plot.flat:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num)
                self.live_lines[plot.name] = self.axes[plot.name].plot([], [], **plot.properties)[0]
            else:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num, projection='3d')
                self.live_lines[plot.name] = self.axes[plot.name].plot([], [], [], **plot.properties)[0]

            plot_num += 1

        self.closed = False
        plt.show(block=False)

    def plot(self):
        if self.closed:
            return False

        for plot in self.robot_plots:
            if plot.should_skip():
                self.should_skip = True
                continue

            self.live_lines[plot.name].set_xdata(plot.data[0])
            self.live_lines[plot.name].set_ydata(plot.data[1])

            if not plot.flat:
                self.live_lines[plot.name].set_3d_properties(plot.data[2])

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

    def handle_close(self):
        self.close()

    def close(self):
        if not self.closed:
            self.closed = True
            plt.ioff()
            plt.gcf()
            plt.close('all')


class StaticPlotter:
    def __init__(self, parser, plot_info, enable_3d, use_pickled_data):
        self.plot_data = {}
        self.plot_info = plot_info
        self.plot_enabled = defaultdict(lambda: False)
        self.enable_3d = enable_3d

        for plot_name in self.plot_info.keys():
            self.set_default_value(plot_name, "label", "")
            self.set_default_value(plot_name, "color", "")
            self.set_default_value(plot_name, "alpha", 1)
            self.set_default_value(plot_name, "markersize", 1)
            self.set_default_value(plot_name, "line_segments", False)
            self.set_default_value(plot_name, "log_based_plot", True)
            self.set_default_value(plot_name, "line_seg_freq", 1)
            self.set_default_value(plot_name, "skip_count", 0)
            self.plot_info[plot_name]["line_seg_counter"] = 0

            self.plot_enabled[plot_name] = True

            if self.plot_info[plot_name]["line_segments"]:
                self.plot_data[plot_name] = []
            else:
                self.plot_data[plot_name] = \
                    [[] for _ in range(3 if self.enable_3d else 2)]

        self.timestamps = []

        self.percent = 0
        self.prev_percent = 0

        # ----- pickled simulation properties -----

        self.use_pickled_data = use_pickled_data

        self.pickle_file_name = parser.file_name[:-len(log_file_type)] + pickle_file_type
        self.log_pickle_dir = project.interpret_dir(pickled_sim_directory)

        if self.use_pickled_data:
            if os.path.isfile(os.path.join(self.log_pickle_dir, self.pickle_file_name)):
                with open(os.path.join(self.log_pickle_dir, self.pickle_file_name), 'rb') as pickle_file:
                    self.plot_data, self.enable_3d = pickle.load(pickle_file)
                print("Using picked simulation")
            else:
                self.use_pickled_data = False

        # ----- initialize figures -----

        self.fig = plt.figure(0)

        if self.enable_3d:
            # self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))
            self.ax = p3.Axes3D(self.fig)
        else:
            self.ax = self.fig.gca()
        self.fig.canvas.set_window_title(parser.file_name[:-4])

    def draw_dot(self, x, y, z=0, color='black'):
        if self.enable_3d:
            self.ax.append([x], [y], [z], 'o', color=color, markersize=10)
        else:
            self.ax.append(x, y, 'o', color=color, markersize=10)

    def set_default_value(self, data_name, key, default):
        if key not in self.plot_info[data_name].keys():
            self.plot_info[data_name][key] = default

    def append_data(self, plot_name, x, y, z=None):
        self.plot_data[plot_name][0].append(x)
        self.plot_data[plot_name][1].append(y)
        if self.enable_3d:
            assert z is not None
            self.plot_data[plot_name][2].append(z)

    def step(self, index, timestamp, whoiam, robot_object):
        pass

    def angled_line_segment(self, plot_option, x0, y0, angle, length):
        if not self.enable_3d:
            if self.plot_enabled[plot_option]:
                cycler = self.plot_info[plot_option]["line_seg_counter"] % \
                         self.plot_info[plot_option]["line_seg_freq"]
                if cycler == 0:
                    length *= 1E-4  # scale down to GPS size
                    x1 = x0 + length * math.cos(angle)
                    y1 = y0 + length * math.sin(angle)
                    self.xy_line_segment(plot_option, x0, y0, x1, y1, cycler)

                self.plot_info[plot_option]["line_seg_counter"] += 1

    def xy_line_segment(self, plot_option, x0, y0, x1, y1, cycler=None):
        if self.plot_enabled[plot_option]:
            if cycler is None:
                cycler = self.plot_info[plot_option]["line_seg_counter"] % \
                         self.plot_info[plot_option]["line_seg_freq"]
                self.plot_info[plot_option]["line_seg_counter"] += 1

            if cycler == 0:
                self.plot_data[plot_option].append((x0, x1))
                self.plot_data[plot_option].append((y0, y1))
                self.plot_data[plot_option].append(
                    self.plot_info[plot_option]["color"])

                if not self.plot_info[plot_option]["line_segments"]:
                    self.plot_info[plot_option]["line_segments"] = True

    def fill_data_array(self, plot_option, data_array):
        pass

    def graph_limits(self, x0, x1, y0, y1, z0=None, z1=None):
        if x0 < x1:
            self.ax.set_xlim([x0, x1])
        else:
            self.ax.set_xlim([x1, x0])

        if y0 < y1:
            self.ax.set_ylim([y0, y1])
        else:
            self.ax.set_ylim([y1, y0])

        if self.enable_3d:
            if z0 < z1:
                self.ax.set_zlim([z0, z1])
            else:
                self.ax.set_zlim([z1, z0])

    def plot(self):
        with open(os.path.join(self.log_pickle_dir, self.pickle_file_name), 'wb') as pickle_file:
            pickle.dump((self.plot_data, self.enable_3d), pickle_file, pickle.HIGHEST_PROTOCOL)

        if self.plot_data.keys() != self.plot_info.keys():
            print("WARNING: plot options in current simulator don't match "
                  "options found in the pickled simulation.\nConsider "
                  "setting use_pickled_data to False and running the "
                  "simulation again.")

        for plot_option, data_array in self.plot_data.items():
            if plot_option in self.plot_info:
                data_info = self.plot_info[plot_option]
                if not data_info["log_based_plot"]:
                    self.fill_data_array(plot_option, data_array)

                if data_info["skip_count"] > 0:
                    for index in range(len(data_array)):
                        data_array[index] = \
                            data_array[index][::data_info["skip_count"]]

                if data_info["line_segments"]:
                    if not self.enable_3d:
                        plt.plot(*data_array)
                else:
                    if not self.enable_3d:
                        plt.plot(data_array[0], data_array[1],
                                 data_info["color"],
                                 label=data_info["label"],
                                 markersize=data_info["markersize"],
                                 alpha=data_info["alpha"])
                    else:
                        self.ax.append(data_array[0], data_array[1],
                                       data_array[2], color=data_info["color"],
                                       linewidth=1,
                                       antialiased=False,
                                       label=data_info["label"])
            else:
                print(plot_option, "not found in pickled data, skipping")

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()
