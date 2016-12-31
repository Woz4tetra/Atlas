import math
import traceback
from collections import defaultdict

import mpl_toolkits.mplot3d.axes3d as p3
import numpy as np
from matplotlib import pyplot as plt

from atlasbuggy.logs import *

pickled_sim_directory = ":simulations"


class LivePlotter:
    is_interactive = False
    fig_num = 0

    def __init__(self, x_range, y_range, z_range=None, color='blue', linestyle='-', marker=None):
        if not LivePlotter.is_interactive:
            plt.ion()
            LivePlotter.is_interactive = True

        self.min_x = x_range[0]
        self.max_x = x_range[1]

        self.min_y = y_range[0]
        self.max_y = y_range[1]

        self.x_data = np.array([])
        self.y_data = np.array([])

        self.enable_3d = True if z_range is not None else False

        self.my_fig_num = LivePlotter.fig_num

        self.fig = plt.figure(LivePlotter.fig_num)
        LivePlotter.fig_num += 1
        self.fig.canvas.mpl_connect('close_event', lambda event: self.handle_close())
        self.is_running = True

        if self.enable_3d:
            self.min_z = z_range[0]
            self.max_z = z_range[1]
            self.z_data = np.array([])

            # self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))
            self.ax = p3.Axes3D(self.fig)

            self.plot_data = \
                self.ax.plot([0], [0], [0], color=color, linestyle=linestyle, marker=marker, markersize=1)[0]
            self.current_data_point = \
                self.ax.plot([0], [0], [0], '.',
                             color='black', markersize=10)[0]
        else:
            # self.ax = self.fig.add_subplot(111)
            self.ax = self.fig.gca()
            self.plot_data = self.ax.plot(0, 0, color=color, linestyle=linestyle, marker=marker)[0]
            self.current_data_point = \
                self.ax.plot(0, 0, 'o', color='black', markersize=10)[0]

        super(LivePlotter, self).__init__()

    def handle_close(self):
        # print("Plot window closed")
        self.is_running = False
        self.close()

    def plot(self, x, y, z=None):
        if not self.is_running:
            return False
        try:
            self.fig = plt.figure(self.my_fig_num)

            self.x_data = np.append(self.x_data, x)
            self.y_data = np.append(self.y_data, y)

            self.plot_data.set_xdata(self.x_data)
            self.plot_data.set_ydata(self.y_data)

            self.current_data_point.set_xdata([x])
            self.current_data_point.set_ydata([y])

            if x < self.min_x:
                self.min_x = x
            if x > self.max_x:
                self.max_x = x

            if y < self.min_y:
                self.min_y = y
            if y > self.max_y:
                self.max_y = y

            if self.enable_3d:
                assert z is not None

                if z < self.min_z:
                    self.min_z = z
                if z > self.max_z:
                    self.max_z = z

                self.z_data = np.append(self.z_data, z)

                self.current_data_point.set_3d_properties([z])

                self.plot_data.set_3d_properties(self.z_data)

                self.ax.set_xlim3d([self.min_x, self.max_x])
                self.ax.set_ylim3d([self.min_y, self.max_y])
                self.ax.set_zlim3d([self.min_z, self.max_z])
            else:
                plt.axis([self.min_x, self.max_x, self.min_y, self.max_y])

            self.fig.canvas.draw()
            plt.pause(0.01)  # can't be less than ~0.005

            return True
        except:
            # print("plot closing")
            traceback.print_exc()

            self.is_running = False
            return False

    def close(self):
        if self.is_running:
            plt.ioff()
            plt.close('all')
            plt.gcf()


class PostPlotter:
    def __init__(self, file_name, directory, plot_info, enable_3d,
                 use_pickled_data, start_index=0, end_index=-1, *robot_objects):
        self.simulated_ports = {}
        for robot_object in robot_objects:
            self.simulated_ports[robot_object.whoiam] = robot_object

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

        self.parser = Parser(file_name, directory, start_index, end_index)

        self.timestamps = []

        self.percent = 0
        self.prev_percent = 0

        # ----- pickled simulation properties -----

        self.use_pickled_data = use_pickled_data

        self.pickle_file_name = self.parser.file_name[:-len(log_file_type)] + pickle_file_type
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
        self.fig.canvas.set_window_title(self.parser.file_name[:-4])

    def draw_dot(self, x, y, z=0, color='black'):
        if self.enable_3d:
            self.ax.plot([x], [y], [z], 'o', color=color, markersize=10)
        else:
            self.ax.plot(x, y, 'o', color=color, markersize=10)

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

    def run(self):
        for index, timestamp, whoiam, packet in self.parser:
            if timestamp == -1:
                self.simulated_ports[whoiam].receive_first(packet)
            else:
                self.simulated_ports[whoiam].receive(packet)
                self.step(index, timestamp, whoiam,
                          self.simulated_ports[whoiam])

                self.timestamps.append(timestamp)

                percent = 100 * index / len(self.parser)
                self.percent = int(percent * 10)
                if self.percent != self.prev_percent:
                    self.prev_percent = self.percent
                    print(("%0.1f" % percent) + "%", end='\r')

        print("plotting...")
        self.plot()
        self.close()

    def angled_line_segment(self, plot_option, x0, y0, angle, length,
                            vx=None, vy=None, v_scale=None):
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

    def before_plot(self):
        pass

    def plot(self):
        with open(os.path.join(self.log_pickle_dir, self.pickle_file_name), 'wb') as pickle_file:
            pickle.dump((self.plot_data, self.enable_3d), pickle_file, pickle.HIGHEST_PROTOCOL)

        self.before_plot()

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
                        self.ax.plot(data_array[0], data_array[1],
                                     data_array[2], color=data_info["color"],
                                     linewidth=1,
                                     antialiased=False,
                                     label=data_info["label"])
            else:
                print(plot_option, "not found in pickled data, skipping")

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def close(self):
        pass
