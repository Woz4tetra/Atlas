from matplotlib import pyplot as plt
import math
from collections import defaultdict
from autobuggy.microcontroller.logger import *


class Simulator:
    # TODO: Add integration for CV pipelines
    def __init__(self, file_name, directory, max_speed, plot_info):

        self.plot_data = {}
        self.plot_info = plot_info
        self.plot_enabled = defaultdict(lambda: False)

        for plot_option in self.plot_info.keys():
            self.set_default_value(plot_option, "label", "")
            self.set_default_value(plot_option, "color", "")
            self.set_default_value(plot_option, "columns", 2)
            self.set_default_value(plot_option, "markersize", 1)
            self.set_default_value(plot_option, "line_segments", False)
            self.set_default_value(plot_option, "log_based_plot", True)
            self.set_default_value(plot_option, "line_seg_freq", 1)
            self.plot_info[plot_option]["line_seg_counter"] = 0

            self.plot_enabled[plot_option] = True

            if self.plot_info[plot_option]["line_segments"] or \
                            self.plot_info[plot_option]["columns"] == 1:
                self.plot_data[plot_option] = []
            else:
                self.plot_data[plot_option] = \
                    [[] for _ in range(self.plot_info[plot_option]["columns"])]

        self.parser = Parser(file_name, directory)

        self.timestamps = []

        self.prev_percent = 0
        
        self.max_speed = max_speed

        # ----- initialize figures -----

        self.fig = plt.figure(0)
        self.ax = self.fig.gca()
        self.fig.canvas.set_window_title(
            self.parser.local_dir + self.parser.file_name[:-4])
    
    def draw_starting_dot(self, initial_lat, initial_long):
        self.ax.plot(initial_lat, initial_long, 'o',
                     color='black', markersize=10)

    def set_default_value(self, data_name, key, default):
        if key not in self.plot_info[data_name].keys():
            self.plot_info[data_name][key] = default

    def step(self, index, timestamp, name, values):
        pass

    def run(self):
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

    def plot(self):
        for plot_option, data_array in self.plot_data.items():
            data_info = self.plot_info[plot_option]
            if not data_info["log_based_plot"]:
                self.fill_data_array(plot_option, data_array)

            if data_info["line_segments"]:
                plt.plot(*data_array)
            else:
                if data_info["columns"] == 1:
                    plt.plot(self.timestamps, data_array)
                elif data_info["columns"] == 2:
                    plt.plot(data_array[0], data_array[1], data_info["color"],
                             label=data_info["label"],
                             markersize=data_info["markersize"])

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def close(self):
        pass
