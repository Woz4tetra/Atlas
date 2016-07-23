import math
from matplotlib import pyplot as plt

from buggypi.microcontroller.logger import *
from navigation.buggypi_filter import BuggyPiFilter
from navigation.waypoint_picker import Waypoints

from pprint import pprint


class Plotter:
    def __init__(self, file_name, directory):
        self.counts_per_rotation = 6
        self.wheel_radius = 0.097
        self.front_back_dist = 0.234
        self.max_speed = 0.88
        self.left_angle_limit = 0.81096
        self.right_angle_limit = -0.53719
        self.left_servo_limit = 35
        self.right_servo_limit = -25

        self.checkpoints = get_map("checkpoints")
        self.waypoints = Waypoints(
            -1, self.left_angle_limit, self.right_angle_limit)

        initial_long, initial_lat = self.checkpoints[-1]
        second_long, second_lat = self.checkpoints[0]

        bearing = BuggyPiFilter.get_gps_bearing(
            # -71.420864, 42.427317, -71.420795, 42.427332
            initial_long, initial_lat, second_long, second_lat
        )
        bearing = (-bearing - math.pi / 2) % (2 * math.pi)
        self.filter = BuggyPiFilter(self.counts_per_rotation, self.wheel_radius,
                                    self.front_back_dist,
                                    self.max_speed,
                                    self.left_angle_limit, self.right_angle_limit,
                                    self.left_servo_limit, self.right_servo_limit,
                                    initial_long, initial_lat, bearing)
        print(initial_long, initial_lat, bearing)

        self.parser = Parser(file_name, directory)

        self.prev_imu_time = 0.0
        self.prev_enc_time = 0.0

        self.dt_enc = None
        self.dt_imu = None

        self.heading_arrow_len = 0.00002
        self.arrow_color = 'orange'

        self.bearing_arrow_len = 0.000005

        # ----- data to record -----

        self.state_x = [self.filter.initial_long]
        self.state_y = [self.filter.initial_lat]
        # self.state_heading = [(self.initial_long - 0.0001,
        #                        self.initial_long - 0.0001 +
        #                        self.heading_arrow_len),
        #                       (self.initial_lat + 0.00001,
        #                        self.initial_lat + 0.00001),
        #                       'darkgreen']
        self.state_heading = []

        self.lat_data, self.long_data = [], []  # all gps data
        self.check_lat, self.check_long = [], []  # checkpoint gps points

        self.state_x_gps = [self.filter.initial_long]
        self.state_y_gps = [self.filter.initial_lat]

        self.bearing_data = []

        self.recorded_state_x = []
        self.recorded_state_y = []
        self.recorded_state_heading = []

        self.recorded_heading_counter = 0
        self.recording_heading_freq = 5

        self.heading_counter = 0
        self.heading_freq = 50

        self.waypoint_lines = []
        self.waypoint_color = 'burlywood'
        self.waypoint_counter = 0
        self.waypoint_freq = 100

        # ----- initialize figures -----

        self.fig = plt.figure(0)
        self.ax = self.fig.gca()
        self.ax.plot(self.filter.initial_long, self.filter.initial_lat, 'o', color='black', markersize=10)
        self.fig.canvas.set_window_title(
            self.parser.local_dir + self.parser.file_name[:-4])

        # self.ax.arrow(0, 0, 0.5, 0.5, head_width=0.05, head_length=0.1, fc='k', ec='k')

    def record_waypoints(self, state):
        if self.waypoint_counter % self.waypoint_freq == 0:
            goal_x, goal_y = self.waypoints.get_goal(state)
        #     self.waypoint_lines.append((state["x"], goal_x))
        #     self.waypoint_lines.append((state["y"], goal_y))
        #     self.waypoint_lines.append(self.waypoint_color)
        #     self.ax.arrow(state["x"], state["y"], goal_x, goal_y, head_width=0.0000001, head_length=0.0000001, fc='k', ec='k')
            self.ax.annotate("",
                        xy=(state["x"], state["y"]), xycoords='data',
                        xytext=(goal_x, goal_y), textcoords='data',
                        arrowprops=dict(arrowstyle="->",
                                        connectionstyle="arc",
                                        linewidth=4),
                        )

        self.waypoint_counter += 1


    def record_state_data(self, state, state_x_data, state_y_data,
                          state_heading_data, heading_counter, heading_color,
                          frequency):
        state_x_data.append(state["x"])
        state_y_data.append(state["y"])

        if heading_counter % frequency == 0 and \
                        len(state_x_data) > 0 and len(state_y_data) > 0:
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

    def step(self, index, timestamp, name, values):
        if name == "gps":
            state = self.filter.update_gps(
                timestamp, values["long"], values["lat"])

            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

            self.state_x_gps.append(state["x"])
            self.state_y_gps.append(state["y"])

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

            self.record_waypoints(state)

        elif name == "imu":
            state = self.filter.update_imu(timestamp, values["yaw"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

            self.record_waypoints(state)
        elif name == "encoder":
            state = self.filter.update_encoder(timestamp, values["counts"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

            self.record_waypoints(state)

        elif name == "servo":
            self.filter.update_servo(values)
        elif name == "motors":
            self.filter.update_motors(values)
        elif name == "checkpoint":
            long, lat = self.checkpoints[values['num']]
            self.check_long.append(long)
            self.check_lat.append(lat)
        elif name == "state":
            self.recorded_heading_counter = \
                self.record_state_data(values, self.recorded_state_x,
                                       self.recorded_state_y,
                                       self.recorded_state_heading,
                                       self.recorded_heading_counter,
                                       'purple', self.recording_heading_freq)

        percent = 100 * index / len(self.parser)
        print(("%0.4f" % percent) + "%", end='\r')

    def static_plot(self, plot_recorded_state, plot_calculated_state):
        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)

        print("plotting...")
        plt.plot(self.long_data, self.lat_data, "r", label="GPS")
        plt.plot(self.check_long, self.check_lat, "o", label="Checkpoints")
        plt.plot(*self.bearing_data)

        if plot_calculated_state:
            plt.plot(self.state_x_gps, self.state_y_gps, "ro",
                     label="GPS updated",
                     markersize=4)
            plt.plot(self.state_x, self.state_y, 'g', label="state xy")
            plt.plot(*self.state_heading)
        if plot_recorded_state:
            plt.plot(self.recorded_state_x, self.recorded_state_y, 'g',
                     label="state xy")
            plt.plot(*self.recorded_state_heading)

        plt.plot(*self.waypoint_lines)

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def write_maps(self, skip=1):
        plt.close()
        print("You are about to create map based on the current plot. "
              "Continue? (y/n)", end="")

        proceed = None
        while proceed != "" and proceed != "y" and proceed != "n":
            proceed = input(": ").lower()

        if proceed != "n":
            map_file = open("%s%s map.txt" % (project.get_dir(":maps"),
                                              self.parser.file_name[:-4]), "w+")
            map_gpx_file = open("%s%s gpx map.gpx" %
                                (project.get_dir(":gpx"),
                                 self.parser.file_name[:-4]), "w+")

            assert len(self.state_x) == len(self.state_y)
            map_file.write("long, lat\n")
            for index in range(0, len(self.state_x), skip):
                map_file.write("%s, %s\n" % (
                    self.state_x[index], self.state_y[index]))

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
                    min(self.state_y), min(self.state_x), max(self.state_y),
                    max(self.state_x)
                ))
            map_gpx_file.write("<trk><name>checkpoints</name><trkseg>\n")
            for index in range(0, len(self.state_x), skip):
                map_gpx_file.write('<trkpt lat="%s" lon="%s"></trkpt>\n' % (
                    self.state_y[index], self.state_x[index]))

            map_gpx_file.write('</trkseg></trk></gpx>\n')

            map_file.close()
            map_gpx_file.close()


if len(sys.argv) == 2:
    file_name = sys.argv[1]
    directory = None
elif len(sys.argv) == 3:
    file_name = sys.argv[1]
    directory = sys.argv[2]
else:
    file_name = -3
    directory = "Jul 22 2016"
    # file_name = ":random"
    # directory = ":random"
    # file_name = 'Mon Jul 11 19;50;34 2016.txt'
    # directory = 'Jul 11 2016'
try:
    file_name = int(file_name)
except ValueError:
    pass

plotter = Plotter(file_name, directory)
plotter.static_plot(plot_recorded_state=False, plot_calculated_state=True)
# plotter.write_maps(10)
