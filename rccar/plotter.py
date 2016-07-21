import sys
import math

from buggypi.microcontroller.logger import get_map
from buggypi.robot_plotter import RobotPlotter
from navigation.buggypi_filter import BuggyPiFilter


class Plotter(RobotPlotter):
    def __init__(self):
        if len(sys.argv) == 2:
            file_name = sys.argv[1]
            directory = None
        elif len(sys.argv) == 3:
            file_name = sys.argv[1]
            directory = sys.argv[2]
        else:
            file_name = 0
            directory = "Jul 11 2016"
            # file_name = ":random"
            # directory = ":random"
            # file_name = 'Mon Jul 11 19;50;34 2016.txt'
            # directory = 'Jul 11 2016'
        try:
            file_name = int(file_name)
        except ValueError:
            pass

        self.counts_per_rotation = 6
        self.wheel_radius = 0.097
        self.front_back_dist = 0.234
        self.max_speed = 0.88
        self.left_angle_limit = 0.81096
        self.right_angle_limit = -0.53719
        self.left_servo_limit = 35
        self.right_servo_limit = -25

        self.checkpoints = get_map("checkpoints")
        initial_long, initial_lat = self.checkpoints[0]
        filter = BuggyPiFilter(self.counts_per_rotation, self.wheel_radius,
                               self.front_back_dist,
                               self.max_speed,
                               self.left_angle_limit, self.right_angle_limit,
                               self.left_servo_limit, self.right_servo_limit,
                               initial_long, initial_lat, 0)

        super(Plotter, self).__init__(filter, file_name, directory)

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

        elif name == "imu":
            state = self.filter.update_imu(timestamp, values["yaw"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)
        elif name == "encoder":
            state = self.filter.update_encoder(timestamp, values["counts"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

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
        print(str(int(percent)) + "%", end='\r')

Plotter().static_plot(plot_recorded_state=True, plot_calculated_state=True)
