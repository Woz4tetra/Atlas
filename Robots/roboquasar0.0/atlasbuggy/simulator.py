from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
import math
from collections import defaultdict
from atlasbuggy.microcontroller.logger import *
import pickle

pickled_sim_directory = ":simulations"


class Simulator:
    # TODO: Add integration for CV pipelines
    def __init__(self, file_name, directory, max_speed, plot_info,
                 enable_3d=False, use_pickled_data=False,
                 start_index=0, end_index=-1):
        self.enable_3d = enable_3d

        self.plot_data = {}
        self.plot_info = plot_info
        self.plot_enabled = defaultdict(lambda: False)

        for plot_option in self.plot_info.keys():
            self.set_default_value(plot_option, "label", "")
            self.set_default_value(plot_option, "color", "")
            self.set_default_value(plot_option, "alpha", 1)
            self.set_default_value(plot_option, "markersize", 1)
            self.set_default_value(plot_option, "line_segments", False)
            self.set_default_value(plot_option, "log_based_plot", True)
            self.set_default_value(plot_option, "line_seg_freq", 1)
            self.set_default_value(plot_option, "skip_count", 0)
            self.plot_info[plot_option]["line_seg_counter"] = 0

            self.plot_enabled[plot_option] = True

            if self.plot_info[plot_option]["line_segments"]:
                self.plot_data[plot_option] = []
            else:
                self.plot_data[plot_option] = \
                    [[] for _ in range(3 if self.enable_3d else 2)]

        self.parser = Parser(file_name, directory, start_index, end_index)
        self.start_time = self.parser.data[0][0]

        self.timestamps = []

        self.prev_percent = 0

        self.max_speed = max_speed

        # ----- pickled simulation properties -----

        self.use_pickled_data = use_pickled_data

        self.pickle_file_name = self.parser.file_name[
                                :-len(log_file_type)] + pickle_file_type
        self.log_pickle_dir = project.interpret_dir(
            pickled_sim_directory)

        if self.use_pickled_data:
            if os.path.isfile(self.log_pickle_dir + self.pickle_file_name):
                with open(self.log_pickle_dir + self.pickle_file_name,
                          'rb') as pickle_file:
                    self.plot_data, self.enable_3d = pickle.load(pickle_file)
                print("Using picked simulation")
            else:
                self.use_pickled_data = False

        # ----- initialize figures -----

        if self.enable_3d:
            self.fig, self.ax = plt.subplots(subplot_kw=dict(projection='3d'))
        else:
            self.fig = plt.figure(0)
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

    def step(self, index, timestamp, name, values):
        pass

    def run(self):
        if not self.use_pickled_data:
            for index, timestamp, name, values in self.parser:
                self.step(index, timestamp, name, values)

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
                    if vx is not None and vy is not None and v_scale is not None:
                        speed = (vx ** 2 + vy ** 2) ** 0.5
                        speed = speed * v_scale + v_scale
                        percent_speed = abs(
                            self.max_speed - speed) / self.max_speed
                        length *= percent_speed
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
        with open(self.log_pickle_dir + self.pickle_file_name,
                  'wb') as pickle_file:
            pickle.dump((self.plot_data, self.enable_3d), pickle_file,
                        pickle.HIGHEST_PROTOCOL)

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
