import math
import numpy as np
from matplotlib import pyplot as plt
from pykalman import KalmanFilter

from autobuggy.microcontroller.logger import *
from navigation.rccar_filter import RcCarFilter
from navigation.waypoint_picker import Waypoints
from standard_params import standard_params
from collections import defaultdict


class Plotter:
    def __init__(self, file_name, directory, map_name, map_dir=None,
                 *plot_options):
        project.set_project_dir("rccar")

        self.map_name = map_name
        self.map_dir = map_dir

        self.plot_options = defaultdict(lambda: False)
        for option in plot_options:
            self.plot_options[option] = True

        self.checkpoints = get_map("checkpoints.txt")
        self.waypoints = Waypoints(
            self.map_name, map_dir=map_dir
        )

        initial_long, initial_lat = self.checkpoints[0]
        second_long, second_lat = self.checkpoints[1]

        initial_heading = RcCarFilter.get_gps_bearing(
            second_long, second_lat, initial_long, initial_lat
        )

        self.filter = RcCarFilter(
            standard_params['counts_per_rotation'],
            standard_params['wheel_radius'],
            standard_params['front_back_dist'],
            standard_params['max_speed'],
            standard_params['left_angle_limit'],
            standard_params['right_angle_limit'],
            standard_params['left_servo_limit'],
            standard_params['right_servo_limit'],
            initial_long, initial_lat, initial_heading
        )
        print(initial_long, initial_lat, initial_heading)

        self.parser = Parser(file_name, directory)

        self.prev_imu_time = 0.0
        self.prev_enc_time = 0.0

        self.dt_enc = None
        self.dt_imu = None

        self.heading_arrow_len = 0.00005
        self.arrow_color = 'orange'

        self.bearing_arrow_len = 0.000005

        # ----- data to record -----

        self.state_x = [self.filter.initial_long]
        self.state_y = [self.filter.initial_lat]
        self.state_heading = []

        self.lat_data, self.long_data = [], []  # all gps data
        self.check_lat, self.check_long = [], []  # checkpoint gps points

        # put red dots on the state line where the GPS updated
        self.state_x_gps = [self.filter.initial_long]
        self.state_y_gps = [self.filter.initial_lat]

        self.bearing_data = []

        self.recorded_state_x = []
        self.recorded_state_y = []
        self.recorded_state_heading = []

        self.recorded_heading_counter = 0
        self.recorded_heading_freq = 10

        self.heading_counter = 0
        self.heading_freq = 5

        self.waypoint_lines = []
        self.waypoint_color = 'burlywood'
        self.waypoint_counter = 0
        self.waypoint_freq = 75
        self.percent = 0
        self.prev_percent = 0
        self.checkpoint_lines = []

        self.encoder_x_data = []
        self.encoder_y_data = []
        self.encoder_color = 'darkblue'

        self.measurement = np.ma.array(np.zeros(len(
            self.filter.observation_matrix)))
        self.pykf_measurements = np.ma.array(self.measurement)

        self.sensors_updated = False
        self.gps_updated = False
        self.imu_updated = False
        self.enc_updated = False

        self.current_long = initial_long
        self.current_lat = initial_lat
        self.current_yaw = None
        self.current_enc = 0

        # ----- initialize figures -----

        self.fig = plt.figure(0)
        self.ax = self.fig.gca()
        self.ax.plot(self.filter.initial_long, self.filter.initial_lat, 'o',
                     color='black', markersize=10)
        self.fig.canvas.set_window_title(
            self.parser.local_dir + self.parser.file_name[:-4])

    def record_waypoints(self, state):
        if self.waypoint_counter % self.waypoint_freq == 0:
            goal_x, goal_y = self.waypoints.get_goal(state)
            self.ax.annotate("",
                             xy=(goal_x, goal_y), xycoords='data',
                             xytext=(state["x"], state["y"]), textcoords='data',
                             arrowprops=dict(arrowstyle="->",
                                             connectionstyle="arc",
                                             linewidth=1),
                             )

        self.waypoint_counter += 1

    def record_state_data(self, state, state_x_data, state_y_data,
                          state_heading_data, heading_counter, heading_color,
                          frequency):
        state_x_data.append(state["x"])
        state_y_data.append(state["y"])

        if heading_counter % frequency == 0 and \
                        len(state_x_data) > 0 and len(state_y_data) > 0 and \
                self.plot_options["plot_heading_lines"]:
            x0 = state_x_data[-1]
            y0 = state_y_data[-1]
            speed = (state["vx"] ** 2 + state["vy"] ** 2) ** 0.5
            speed = speed * 0.75 + 0.75
            percent_speed = abs(
                self.filter.max_speed - speed) / self.filter.max_speed
            x1 = x0 + self.heading_arrow_len * percent_speed * math.cos(
                state["angle"])
            y1 = y0 + self.heading_arrow_len * percent_speed * math.sin(
                state["angle"])
            state_heading_data.append((x0, x1))
            state_heading_data.append((y0, y1))
            state_heading_data.append(heading_color)

        return heading_counter + 1

    def parse_gps(self, timestamp, values):
        if self.plot_options["plot_calculated_state"]:
            # state = self.filter.update_gps(
            #     timestamp, values["long"], values["lat"])
            #
            # self.heading_counter = \
            #     self.record_state_data(state, self.state_x, self.state_y,
            #                            self.state_heading,
            #                            self.heading_counter,
            #                            self.arrow_color, self.heading_freq)
            #
            # if self.plot_options["plot_state_gps_dots"]:
            #     self.state_x_gps.append(state["x"])
            #     self.state_y_gps.append(state["y"])

            if self.plot_options["determine_matrices"]:
                self.measurement[0] = self.filter.measurement[0]
                self.measurement[1] = self.filter.measurement[1]
                self.measurement[2] = self.filter.measurement[2]

        if self.plot_options["plot_gps"]:
            self.long_data.append(values["long"])
            self.lat_data.append(values["lat"])

            x0 = values["long"]
            y0 = values["lat"]
            x1 = x0 + self.bearing_arrow_len * math.cos(
                self.filter.gps_bearing)
            y1 = y0 + self.bearing_arrow_len * math.sin(
                self.filter.gps_bearing)
            self.bearing_data.append((x0, x1))
            self.bearing_data.append((y0, y1))
            self.bearing_data.append('orange')

        self.current_long = values["long"]
        self.current_lat = values["lat"]

        self.sensors_updated = True
        self.gps_updated = True

    def parse_imu(self, timestamp, values):
        # if self.plot_options["plot_calculated_state"]:
        #     state = self.filter.update_imu(timestamp, values["yaw"])
        #     self.heading_counter = \
        #         self.record_state_data(state, self.state_x, self.state_y,
        #                                self.state_heading,
        #                                self.heading_counter,
        #                                self.arrow_color, self.heading_freq)
        #
        #     if self.plot_options["plot_waypoints"]:
        #         self.record_waypoints(state)
        #
        #     if self.plot_options["determine_matrices"]:
        #         self.measurement[5] = self.filter.measurement[5]
        self.current_yaw = values["yaw"]

        self.sensors_updated = True
        self.imu_updated = True

    def parse_encoder(self, timestamp, values):
        # if self.plot_options["plot_calculated_state"]:
        #     state = self.filter.update_encoder(timestamp, values["counts"])
        #     self.heading_counter = \
        #         self.record_state_data(state, self.state_x, self.state_y,
        #                                self.state_heading,
        #                                self.heading_counter,
        #                                self.arrow_color, self.heading_freq)
        #     if self.plot_options["plot_encoder_position"]:
        #         x, y = self.filter.xy_meters_to_gps(self.filter.enc_x,
        #                                             self.filter.enc_y)
        #         self.encoder_x_data.append(math.degrees(x))
        #         self.encoder_y_data.append(math.degrees(y))
        #     if self.plot_options["plot_waypoints"]:
        #         self.record_waypoints(state)
        #
        #     if self.plot_options["determine_matrices"]:
        #         self.measurement[3] = self.filter.measurement[3]
        #         self.measurement[4] = self.filter.measurement[4]
        #         self.measurement[6] = self.filter.measurement[6]
        #         self.measurement[7] = self.filter.measurement[7]

        self.current_enc = values["counts"]

        self.sensors_updated = True
        self.enc_updated = True

    def plot_checkpoint_line(self, x0, y0, x1, y1):
        self.checkpoint_lines.append((x0, x1))
        self.checkpoint_lines.append((y0, y1))
        self.checkpoint_lines.append('blue')

    def step(self, index, timestamp, name, values):
        if name == "gps":
            self.parse_gps(timestamp, values)

        elif name == "imu":
            self.parse_imu(timestamp, values)

        elif name == "encoder":
            self.parse_encoder(timestamp, values)

        elif name == "servo":
            if self.plot_options["plot_calculated_state"]:
                self.filter.update_servo(values)
        elif name == "motors":
            if self.plot_options["plot_calculated_state"]:
                self.filter.update_motors(values)
        elif name == "state":
            if self.plot_options["plot_recorded_state"]:
                self.recorded_heading_counter = \
                    self.record_state_data(values, self.recorded_state_x,
                                           self.recorded_state_y,
                                           self.recorded_state_heading,
                                           self.recorded_heading_counter,
                                           'purple', self.recorded_heading_freq)
                if self.plot_options["waypoints"]:
                    self.record_waypoints(values)
        elif name == "checkpoint":
            x = self.checkpoints[values['num']][0]
            y = self.checkpoints[values['num']][1]
            if self.plot_options["plot_calculated_state"]:
                self.plot_checkpoint_line(
                    x, y, self.state_x[-1], self.state_y[-1])
            if self.plot_options["plot_encoder_position"]:
                self.plot_checkpoint_line(
                    x, y, self.encoder_x_data[-1], self.encoder_y_data[-1])
            if self.plot_options["plot_gps"]:
                self.plot_checkpoint_line(
                    x, y, self.long_data[-1], self.lat_data[-1])

        if self.sensors_updated:
            if self.plot_options["determine_matrices"]:
                self.pykf_measurements = np.vstack((self.pykf_measurements,
                                                    self.measurement))
                # if not self.gps_updated:
                #     self.pykf_measurements[-1] = np.ma.masked
                #     print(repr(self.pykf_measurements))

            self.sensors_updated = False
            self.gps_updated = False
            self.imu_updated = False
            self.enc_updated = False

        if index % 15 == 0:
            state = self.filter.update_all(
                timestamp, self.current_enc, self.current_yaw,
                self.current_long, self.current_lat
            )
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading,
                                       self.heading_counter,
                                       self.arrow_color, self.heading_freq)
            if self.plot_options["plot_waypoints"]:
                self.record_waypoints(state)

            if self.plot_options["plot_encoder_position"]:
                x, y = self.filter.xy_meters_to_gps(self.filter.enc_x,
                                                    self.filter.enc_y)
                self.encoder_x_data.append(math.degrees(x))
                self.encoder_y_data.append(math.degrees(y))

        percent = 100 * index / len(self.parser)
        self.percent = int(percent * 10)
        if self.percent != self.prev_percent:
            self.prev_percent = self.percent
            print(("%0.1f" % percent) + "%", end='\r')

    def static_plot(self):
        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)

        print("plotting...")
        if self.plot_options["plot_map"]:
            gps_map = np.array(self.waypoints.map)
            plt.plot(gps_map[:, 0], gps_map[:, 1], 'fuchsia', label="map")

        if self.plot_options["plot_gps"]:
            plt.plot(self.long_data, self.lat_data, "r", label="GPS")
            plt.plot(*self.bearing_data)

        if self.plot_options["plot_checkpoints"]:
            checkpoints_map = np.array(self.checkpoints)
            plt.plot(checkpoints_map[:, 0], checkpoints_map[:, 1], 'o',
                     label="checkpoints")

        if self.plot_options["plot_calculated_state"]:
            if self.plot_options["plot_state_gps_dots"]:
                plt.plot(self.state_x_gps, self.state_y_gps, "ro",
                         label="GPS updated",
                         markersize=4)
            plt.plot(self.state_x, self.state_y, 'g', label="state")
            if self.plot_options["plot_heading_lines"]:
                plt.plot(*self.state_heading)

        if self.plot_options["plot_recorded_state"]:
            plt.plot(self.recorded_state_x, self.recorded_state_y, 'g',
                     label="recorded state")
            if self.plot_options["plot_heading_lines"]:
                plt.plot(*self.recorded_state_heading)

        if self.plot_options["plot_waypoints"]:
            plt.plot(*self.waypoint_lines)

        if self.plot_options["plot_checkpoint_lines"]:
            plt.plot(*self.checkpoint_lines)

        if self.plot_options["plot_encoder_position"]:
            plt.plot(self.encoder_x_data, self.encoder_y_data,
                     self.encoder_color, label="encoder position")

        if self.plot_options["determine_matrices"]:
            np.delete(self.pykf_measurements, 0)
            np.delete(self.pykf_measurements, 0)
            pykfilter = KalmanFilter(
                initial_state_mean=self.filter.initial_state,
                observation_matrices=self.filter.observation_matrix,
                transition_covariance=self.filter.process_error_covariance,
                observation_covariance=self.filter.measurement_covariance,
            )
            result = pykfilter.em(X=self.pykf_measurements, n_iter=5)
            print(result.transition_covariance, result.observation_covariance)

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def write_maps(self, skip=1, map_source_x=None, map_source_y=None):
        print("You are about to create map based on the current plot. "
              "Continue? (y/n)", end="")

        proceed = None
        while proceed != "" and proceed != "y" and proceed != "n":
            proceed = input(": ").lower()

        if proceed == "y":
            if map_source_x is None:
                map_source_x = self.state_x
            if map_source_y is None:
                map_source_y = self.state_y
            assert len(map_source_x) == len(map_source_y)

            map_dir = project.get_dir(":maps") + self.parser.file_name[:-4]
            gpx_map_dir = project.get_dir(":gpx") + self.parser.file_name[:-4]

            map_file = open("%s map.txt" % map_dir, "w+")
            map_gpx_file = open("%s gpx map.gpx" % gpx_map_dir, "w+")

            assert len(map_source_x) == len(map_source_y)
            map_file.write("long, lat\n")
            for index in range(0, len(map_source_x), skip):
                map_file.write("%s, %s\n" % (
                    map_source_x[index], map_source_y[index]))

            map_gpx_file.write("""<?xml version="1.0" encoding="ISO-8859-1" standalone="no" ?>\n<metadata>
        <gpx creator="WTracks" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.topografix.com/GPX/1/1" version="1.1" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
        <metadata>
          <name>checkpoints</name>
          <desc></desc>
          <author><name>WTracks - Online GPX track editor</name></author>
          <link href="https://wtracks.appspot.com/">
            <text>WTracks</text>
            <type>text/html</type>
          </link>
          <time>2016-06-22T19-23-22Z</time>""")
            map_gpx_file.write(
                '<bounds minlat="%s" minlon="%s" maxlat="%s" '
                'maxlon="%s"/></metadata>\n' % (
                    min(map_source_y), min(map_source_x), max(map_source_y),
                    max(map_source_x)
                ))
            map_gpx_file.write("<trk><name>checkpoints</name><trkseg>\n")
            for index in range(0, len(map_source_x), skip):
                map_gpx_file.write('<trkpt lat="%s" lon="%s"></trkpt>\n' % (
                    map_source_y[index], map_source_x[index]))

            map_gpx_file.write('</trkseg></trk></gpx>\n')

            map_file.close()
            map_gpx_file.close()

            print("Map created successfully!", map_dir)
            print("GPX map created successfully!", gpx_map_dir)
        else:
            print("Skipping map creation")


if len(sys.argv) == 2:
    file_name = sys.argv[1]
    directory = None
    map_name = -1
    map_dir = ":gpx"
elif len(sys.argv) == 3:
    file_name, directory = sys.argv[1:]
    map_name = -1
    map_dir = ":gpx"
elif len(sys.argv) == 4:
    file_name, directory, map_name = sys.argv[1:]
    map_dir = ":gpx"
elif len(sys.argv) == 5:
    file_name, directory, map_name, map_dir = sys.argv[1:]

else:
    file_name = 0  #":random"
    directory = "Jul 22 2016"  #':random'
    map_name = "test goal track"
    map_dir = None

try:
    file_name = int(file_name)
except ValueError:
    pass

plotter = Plotter(
    file_name, directory, map_name, map_dir,
    "plot_map",
    "plot_gps",
    "plot_checkpoints",
    "plot_calculated_state",
    # "plot_recorded_state",
    # "plot_waypoints",
    # "plot_state_gps_dots",
    "plot_checkpoint_lines",
    # "plot_heading_lines",
    "plot_encoder_position",
    # "determine_matrices",
)
plotter.static_plot()
# plotter.write_maps(300)#, plotter.long_data, plotter.lat_data)
