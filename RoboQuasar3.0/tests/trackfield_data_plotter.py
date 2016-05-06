"""
Written by Ben Warwick

RoboQuasar3.0, written for the Atlas Project (autonomous buggy group)
Version 4/22/2016
=========

A test file meant to make data analysis visual and easy. Hopefully this file
will allow for insights into what's wrong with the system.

"""

import os
import sys

from matplotlib import pyplot as plt
from scipy.misc import imread

sys.path.insert(0, "../")

import config
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.converter import Converter
from analyzers.binder import Binder
from analyzers.logger import get_data

from microcontroller.data import *
from controllers.servo_map import *


class Plotter:
    def __init__(self, plot_type, data_set_name, map_name, sensors,
                 plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
                 plot_kalman, x_lim, y_lim):
        if data_set_name is None:
            data_sets = []
            for root, dirs, files in os.walk(config.get_dir(":logs"),
                                             topdown=False):
                for name in files:
                    if name.endswith(".csv"):
                        data_sets.append(root + "/" + name)

            data_set_name = random.choice(data_sets)
            print("picked:", data_set_name)
            log_index = data_set_name.find("logs")
            self.data_set_name = data_set_name[log_index + len("logs") + 1:]
        else:
            self.data_set_name = data_set_name

        if sensors[0] != "gps long":
            if "gps long" in sensors:
                sensors.remove("gps long")
            sensors.insert(0, "gps long")

        if sensors[1] != "gps lat":
            if "gps lat" in sensors:
                sensors.remove("gps lat")
            sensors.insert(1, "gps lat")

        if sensors[2] != "gps sleep":
            if "gps sleep" in sensors:
                sensors.remove("gps sleep")
            sensors.insert(2, "gps sleep")

        self.timestamps, self.data, self.data_length = get_data(
            self.data_set_name, sensors, density=1)

        self.gps_long = self.data[0]
        self.gps_lat = self.data[1]
        self.gps_sleep = self.data[2]
        for _ in range(3):
            self.data.pop(0)

        # initial_gps = self.gps_long[0], self.gps_lat[0]
        self.binder = Binder(map_name)
        initial_gps = self.binder.map.raw_data[0]

        self.converter = Converter(initial_gps[0], initial_gps[1], 0.000003, 0)

        self.heading_filter = HeadingFilter()
        self.position_filter = PositionFilter()

        if plot_map:
            plt.plot(self.binder.map[:, 0], self.binder.map[:, 1], 'c')
        self.plot_gps, self.plot_goals, self.plot_heading, self.plot_bind, self.plot_kalman = \
            plot_gps, plot_goals, plot_heading, plot_bind, plot_kalman

        self.plot_type = plot_type

        plt.title(self.plot_type + ": " + data_set_name)
        self.axes = plt.gca()
        self.x_lim = x_lim
        self.y_lim = y_lim

        self.error_sum_dist = 0
        self.error_sum_heading = 0

        if y_lim is None:
            self.ymin = float(min(self.binder.map[:, 1])) - 10
            self.ymax = float(max(self.binder.map[:, 1])) + 10
        else:
            self.ymin, self.ymax = y_lim
        plt.ylim([self.ymin, self.ymax])

        if y_lim is None:
            self.xmin = float(min(self.binder.map[:, 0])) - 10
            self.xmax = float(max(self.binder.map[:, 0])) + 10
        else:
            self.xmin, self.xmax = x_lim
        plt.xlim([self.xmin, self.xmax])

        self.trackfield = imread(
            config.get_dir(":logs") + "map background.png")
        height, width = self.trackfield.shape[0:2]

        straight_len_m = 100  # meters
        straight_len_px = 1061.908  # pixels

        px_to_m = straight_len_m / straight_len_px
        start_x = 1157.9  # pixels
        start_y = 1359.54  # pixels
        # check_pt_x = 1300
        # check_pt_y = height - 1400

        plt.imshow(self.trackfield,
                   zorder=0, origin=(0, 0), extent=
                   [-start_x * px_to_m, -start_x * px_to_m + width * px_to_m,
                    -start_y * px_to_m, -start_y * px_to_m + height * px_to_m])

    def update_plot(self, index):
        pass

    def run_plot(self):
        pass

    def distance(self, x0, y0, x1, y1):
        return ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5

    def plot_bind_arrow(self, x0, y0, x1, y1,
                        color='midnightblue', head_width=3):
        self.axes.arrow(x0, y0,
                        x1 - x0, y1 - y0,
                        color=color, head_width=head_width)


class StaticPlotter(Plotter):
    def __init__(self, plot_type, data_set_name, map_name, sensors,
                 plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
                 plot_kalman, x_lim, y_lim):
        self.heading_lines = []
        self.bind_lines = []
        self.gps_xs, self.gps_ys = [], []
        self.kalman_xs, self.kalman_ys = [], []

        super(StaticPlotter, self).__init__(
            plot_type, data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
            plot_kalman, x_lim, y_lim)

    def run_plot(self):
        for index in range(0, self.data_length):
            if self.gps_sleep[index] != self.gps_sleep[index - 1]:
                self.update_plot(index)

        if self.x_lim is not None:
            self.axes.set_xlim([self.x_lim[0], self.x_lim[1]])
        if self.y_lim is not None:
            self.axes.set_ylim([self.y_lim[0], self.y_lim[1]])

        if self.plot_heading:
            plt.plot(*self.heading_lines)
        if self.plot_bind:
            plt.plot(*self.bind_lines)
        if self.plot_gps:
            plt.plot(self.gps_xs, self.gps_ys, 'g')
        if self.plot_kalman:
            plt.plot(self.kalman_xs, self.kalman_ys, 'b')

        print(self.data_set_name, end="\t")
        print(str(self.error_sum_dist / self.data_length) + "\t" + str(
            abs(self.error_sum_heading / self.data_length)))

        plt.show()

    def update_headings(self, x, y, heading, color='r', line_length=10):
        self.heading_lines.append((x, x + line_length * math.cos(heading)))

        self.heading_lines.append((y, y + line_length * math.sin(heading)))
        self.heading_lines.append(color)

    def update_binds(self, x0, y0, x1, y1, color='brown'):
        self.bind_lines.append((x0, x1))
        self.bind_lines.append((y0, y1))
        self.bind_lines.append(color)


class KalmanPlotter(StaticPlotter):
    def __init__(self, data_set_name, map_name,
                 plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
                 plot_kalman, x_lim=None, y_lim=None):
        sensors = ["encoder", "imu yaw"]

        super(KalmanPlotter, self).__init__(
            "kalman plotter", data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
            plot_kalman, x_lim, y_lim
        )

        self.encoder, self.yaw = self.data
        self.bind_x, self.bind_y = 0, 0

    def update_plot(self, index):
        gps_heading, bind_heading = self.converter.convert_heading(
            self.gps_long[index], self.gps_lat[index], self.bind_x, self.bind_y)

        gps_x, gps_y, enc_dist = \
            self.converter.convert_position(
                self.gps_long[index], self.gps_lat[index], self.encoder[index])

        kalman_heading = self.heading_filter.update(
            gps_heading, bind_heading, -self.yaw[index],
            self.gps_sleep[index])
        kalman_x, kalman_y = self.position_filter.update(
            gps_x, gps_y, enc_dist, self.gps_sleep[index],
            kalman_heading)

        self.bind_x, self.bind_y, goal_x, goal_y = self.binder.bind(
            (kalman_x, kalman_y))

        self.error_sum_dist += self.distance(kalman_x, goal_x, kalman_y, goal_y)
        self.error_sum_heading += bind_heading - kalman_heading

        if self.plot_heading:
            self.update_headings(kalman_x, kalman_y, kalman_heading)
        if self.plot_bind:
            self.update_binds(kalman_x, kalman_y, self.bind_x, self.bind_y)
        if self.plot_goals:
            self.plot_bind_arrow(self.bind_x, self.bind_y, goal_x, goal_y)
        if self.plot_kalman:
            self.kalman_xs.append(kalman_x)
            self.kalman_ys.append(kalman_y)
        if self.plot_gps:
            self.gps_xs.append(gps_x)
            self.gps_ys.append(gps_y)


class DataRunPlotter(StaticPlotter):
    def __init__(self, data_set_name, map_name,
                 plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
                 plot_kalman, x_lim=None, y_lim=None):
        sensors = ["encoder", "imu yaw",
                   "bind x", "bind y", "bind heading", "goal x", "goal y",
                   "kalman x", "kalman y", "kalman heading"]

        super(DataRunPlotter, self).__init__(
            "data run plotter", data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading, plot_bind,
            plot_kalman, x_lim, y_lim
        )

        (self.encoder, self.yaw, self.bind_x, self.bind_y, self.bind_heading,
         self.goal_x, self.goal_y, self.kalman_x, self.kalman_y,
         self.kalman_heading) = self.data


    def update_plot(self, index):
        gps_x, gps_y, _ = self.converter.convert_position(
            self.gps_long[index], self.gps_lat[index], 0)

        if self.goal_x is None and self.goal_y is None:
            _, _, goal_x, goal_y = self.binder.bind(
                (self.kalman_x[index], self.kalman_y[index]))
        else:
            goal_x, goal_y = self.goal_x[index], self.goal_y[index]

        if self.bind_heading is None:
            _, bind_heading = self.converter.convert_heading(
                0, 0, self.bind_x[index], self.bind_y[index])
        else:
            bind_heading = self.bind_heading[index]

        self.error_sum_dist += self.distance(
            self.kalman_x[index], goal_x, self.kalman_y[index], goal_y)
        self.error_sum_heading += bind_heading - \
                                  self.kalman_heading[index]

        if self.plot_heading:
            self.update_headings(self.kalman_x[index], self.kalman_y[index],
                                 self.kalman_heading[index])
        if self.plot_bind:
            self.update_binds(self.kalman_x[index], self.kalman_y[index],
                              self.bind_x[index], self.bind_y[index])
        if self.plot_goals:
            self.plot_bind_arrow(self.bind_x[index], self.bind_y[index], goal_x,
                                 goal_y)
        if self.plot_kalman:
            self.kalman_xs.append(self.kalman_x[index])
            self.kalman_ys.append(self.kalman_y[index])
        if self.plot_gps:
            self.gps_xs.append(gps_x)
            self.gps_ys.append(gps_y)


class LivePlotter(Plotter):
    def __init__(self, plot_type, data_set_name, map_name, sensors,
                 plot_gps, plot_map, plot_goals, plot_heading,
                 plot_bind,
                 plot_kalman, x_lim=None, y_lim=None):
        super(LivePlotter, self).__init__(
            plot_type, data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading,
            plot_bind,
            plot_kalman, x_lim, y_lim)

        plt.ion()

    def run_plot(self):
        for index in range(0, self.data_length):
            if self.gps_sleep[index] != self.gps_sleep[
                        index - 1]:
                self.update_plot(index)

                plt.draw()
                plt.pause(0.001)

        print(self.data_set_name, end="\t")
        print(str(
            self.error_sum_dist / self.data_length) + "\t" + str(
            abs(self.error_sum_heading / self.data_length)))

        plt.ioff()
        plt.show()

    def plot_headings(self, x, y, heading, color='r', line_length=10):
        plt.plot([x, x + line_length * math.cos(heading)],
                 [y, y + line_length * math.sin(heading)],
                 color)

    def plot_binds(self, x0, y0, x1, y1, color='brown'):
        plt.plot([x0, x1], [y0, y1], color)


class LiveKalmanPlotter(LivePlotter):
    def __init__(self, data_set_name, map_name,
                 plot_gps, plot_map, plot_goals, plot_heading,
                 plot_bind,
                 plot_kalman, x_lim=None, y_lim=None):
        sensors = ["encoder", "imu yaw"]
        super(LiveKalmanPlotter, self).__init__(
            "live kalman plot", data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading,
            plot_bind,
            plot_kalman, x_lim=x_lim, y_lim=y_lim
        )

        self.prev_bind_x = 0
        self.prev_bind_y = 0

        self.bind_x = 0
        self.bind_y = 0

        self.arrow_color = 0

        self.encoder, self.yaw = self.data

        self.kalman_data = [[], []]

    def update_plot(self, index):
        gps_heading, bind_heading = self.converter.convert_heading(
            self.gps_long[index], self.gps_lat[index], self.bind_x, self.bind_y)

        gps_x0, gps_y0, enc_dist = \
            self.converter.convert_position(
                self.gps_long[index], self.gps_lat[index], self.encoder[index])

        kalman_heading = self.heading_filter.update(
            gps_heading, bind_heading, -self.yaw[index],
            self.gps_sleep[index])
        kalman_x, kalman_y = self.position_filter.update(
            gps_x0, gps_y0, enc_dist, self.gps_sleep[index],
            kalman_heading)

        self.prev_bind_x = self.bind_x
        self.prev_bind_y = self.bind_y
        self.bind_x, self.bind_y, goal_x, goal_y = self.binder.bind(
            (kalman_x, kalman_y))

        # current_state = [self.bind_x, self.bind_y, bind_heading]
        # goal_state = [goal_x, goal_y]
        #
        # print(state_to_angle(current_state, goal_state),
        #       state_to_servo(current_state, goal_state),
        #       bind_heading, kalman_heading)
        #
        # input()

        self.error_sum_dist += self.distance(kalman_x, goal_x, kalman_y, goal_y)
        self.error_sum_heading += bind_heading - kalman_heading

        if self.plot_goals:
            if self.bind_x == self.prev_bind_x and self.bind_y == self.prev_bind_y:
                self.arrow_color += 40
                if self.arrow_color > 255:
                    self.arrow_color = 0
            else:
                self.arrow_color = 0
            self.plot_bind_arrow(self.bind_x, self.bind_y, goal_x, goal_y,
                                 color="#%0.2x%0.2x%0.2x" % (
                                     0, self.arrow_color, self.arrow_color
                                 ))
        if self.plot_heading:
            self.plot_headings(kalman_x, kalman_y, kalman_heading)

            if kalman_x < self.xmin:
                self.xmin = kalman_x - 10
                plt.xlim([self.xmin, self.xmax])
            if kalman_x > self.xmax:
                self.xmax = kalman_x + 10
                plt.xlim([self.xmin, self.xmax])

            if kalman_y < self.ymin:
                self.ymin = kalman_y - 10
                plt.ylim([self.ymin, self.ymax])
            if kalman_y > self.ymax:
                self.ymax = kalman_y + 10
                plt.ylim([self.ymin, self.ymax])
        if self.plot_bind:
            self.plot_binds(kalman_x, kalman_y, self.bind_x, self.bind_y)


class LiveDataRunPlotter(LivePlotter):
    def __init__(self, data_set_name, map_name,
                 plot_gps, plot_map, plot_goals, plot_heading,
                 plot_bind,
                 plot_kalman, x_lim=None, y_lim=None):

        sensors = ["encoder", "imu yaw",
                   "bind x", "bind y", "bind heading", "goal x", "goal y",
                   "kalman x", "kalman y", "kalman heading"]
        super(LiveDataRunPlotter, self).__init__(
            "live data run plot", data_set_name, map_name, sensors,
            plot_gps, plot_map, plot_goals, plot_heading,
            plot_bind,
            plot_kalman, x_lim=x_lim, y_lim=y_lim
        )

        self.prev_bind_x = 0
        self.prev_bind_y = 0

        self.bind_x = 0
        self.bind_y = 0

        self.arrow_color = 0

        (self.encoder, self.yaw, self.bind_x, self.bind_y, self.bind_heading,
         self.goal_x, self.goal_y, self.kalman_x, self.kalman_y,
         self.kalman_heading) = self.data

        self.kalman_data = [[], []]

    def update_plot(self, index):
        self.prev_bind_x = self.bind_x[index]
        self.prev_bind_y = self.bind_y[index]

        gps_x, gps_y, _ = self.converter.convert_position(
            self.gps_long[index], self.gps_lat[index], 0)

        if self.goal_x is None and self.goal_y is None:
            _, _, goal_x, goal_y = self.binder.bind(
                (self.kalman_x[index], self.kalman_y[index]))
        else:
            goal_x, goal_y = self.goal_x[index], self.goal_y[index]

        self.error_sum_dist += self.distance(self.kalman_x[index], goal_x,
                                             self.kalman_y[index], goal_y)
        self.error_sum_heading += self.bind_heading[index] - \
                                  self.kalman_heading[index]

        if self.plot_goals:
            if self.bind_x[index] == self.prev_bind_x and self.bind_y[
                index] == self.prev_bind_y:
                self.arrow_color += 40
                if self.arrow_color > 255:
                    self.arrow_color = 0
            else:
                self.arrow_color = 0
            self.plot_bind_arrow(self.bind_x[index], self.bind_y[index], goal_x,
                                 goal_y,
                                 color="#%0.2x%0.2x%0.2x" % (
                                     0, self.arrow_color, self.arrow_color
                                 ))
        if self.plot_heading:
            self.plot_headings(self.kalman_x[index], self.kalman_y[index],
                               self.kalman_heading[index])

            if self.kalman_x[index] < self.xmin:
                self.xmin = self.kalman_x[index] - 10
                plt.xlim([self.xmin, self.xmax])
            if self.kalman_x[index] > self.xmax:
                self.xmax = self.kalman_x[index] + 10
                plt.xlim([self.xmin, self.xmax])

            if self.kalman_y[index] < self.ymin:
                self.ymin = self.kalman_y[index] - 10
                plt.ylim([self.ymin, self.ymax])
            if self.kalman_y[index] > self.ymax:
                self.ymax = self.kalman_y[index] + 10
                plt.ylim([self.ymin, self.ymax])
        if self.plot_bind:
            self.plot_binds(self.kalman_x[index], self.kalman_y[index],
                            self.bind_x[index], self.bind_y[index])


if __name__ == '__main__':
    print(__doc__)
    LiveKalmanPlotter(
        "Test Day 11/Sat Apr 23 17;42;47 2016.csv",
        # None,
        # "Test Day 5/Sun Apr 10 18;21;57 2016.csv",
        "wtracks map converted.csv",
        plot_gps=True, plot_map=True, plot_goals=True, plot_heading=True,
        plot_bind=True, plot_kalman=True
    ).run_plot()

    # trackfield = imread(
    #     "/Users/Woz4tetra/Desktop/Screen Shot 2016-05-04 at 12.20.56 AM.png")
    #
    # plt.imshow(trackfield, origin=(0, 0), extent=[-1600, 1600, -500, 500])

    plt.show()
