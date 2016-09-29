import sys
import math
import numpy as np
# from autobuggy import project

from autobuggy.simulator import Simulator
from navigation.rccar_filter import RcCarFilter
from standard_params import standard_params
from navigation.waypoint_picker import Waypoints
from autobuggy.microcontroller.logger import get_map


class FilterTest(Simulator):
    def __init__(self, file_name, directory, map_name, map_dir, FilterClass,
                 **plot_info):
        # project.set_project_dir("rccar")
        self.checkpoints = get_map("checkpoints.txt")
        self.waypoints = Waypoints(
            map_name, map_dir=map_dir
        )

        initial_long, initial_lat = self.checkpoints[0]
        second_long, second_lat = self.checkpoints[1]

        initial_heading = FilterClass.get_gps_bearing(
            second_long, second_lat, initial_long, initial_lat
        )

        filter = FilterClass(
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

        super(FilterTest, self).__init__(
            file_name, directory, filter, plot_info
        )

    def fill_data_array(self, plot_option, data_array):
        if plot_option == "map_plot":
            gps_map = np.array(self.waypoints.map)
            data_array[0] = gps_map[:, 0]
            data_array[1] = gps_map[:, 1]

        elif plot_option == "checkpoints_plot":
            checkpoints_map = np.array(self.checkpoints)
            data_array[0] = checkpoints_map[:, 0]
            data_array[1] = checkpoints_map[:, 1]

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.record_imu(timestamp, values)
        elif name == "gps":
            self.record_gps(timestamp, values)
        elif name == "encoder":
            self.record_enc(timestamp, values)
        elif name == "motors":
            self.record_motors(values)
        elif name == "servo":
            self.record_servo(values)
        elif name == "state":
            self.record_state(values, "recorded_state_plot",
                              "recorded_heading_lines_plot")
        elif name == "checkpoint":
            self.record_checkpoint(values)

    def record_checkpoint(self, values):
        check_long, check_lat = self.checkpoints[values["num"]]
        self.xy_line_segment(
            "checkpoint_lines_plot", self.filter.state["x"],
            self.filter.state["y"], check_long, check_lat)

    def record_waypoint(self, state):
        goal_x, goal_y = self.waypoints.get_goal(state)
        self.xy_line_segment(
            "waypoints_plot", state["x"], state["y"], goal_x, goal_y)

    def record_state(self, state, state_plot, heading_lines_plot):
        if self.plot_enabled[state_plot]:
            self.plot_data[state_plot][0].append(state["x"])
            self.plot_data[state_plot][1].append(state["y"])

        self.angled_line_segment(
            heading_lines_plot, state["x"], state["y"], state["angle"],
            0.0001, state["vx"], state["vy"], 0.75)

    def record_imu(self, timestamp, values):
        state = self.filter.update_imu(timestamp, values["yaw"])
        if self.plot_enabled["calculated_state_plot"]:
            self.record_state(state, "calculated_state_plot",
                              "heading_lines_plot")
            self.record_waypoint(state)

    def record_gps(self, timestamp, values):
        state = self.filter.update_gps(timestamp, values["long"], values["lat"])
        if self.plot_enabled["calculated_state_plot"]:
            self.record_state(state, "calculated_state_plot",
                              "heading_lines_plot")
            self.record_waypoint(state)

        if self.plot_enabled["gps_plot"]:
            self.plot_data["gps_plot"][0].append(values["long"])
            self.plot_data["gps_plot"][1].append(values["lat"])

    def record_enc(self, timestamp, values):
        state = self.filter.update_encoder(timestamp, values["counts"])

        if self.plot_enabled["calculated_state_plot"]:
            self.record_state(state, "calculated_state_plot",
                              "heading_lines_plot")
            self.record_waypoint(state)

        if self.plot_enabled["encoder_position_plot"]:
            x, y = self.filter.xy_meters_to_gps(
                self.filter.enc_x, self.filter.enc_y)
            self.plot_data["encoder_position_plot"][0].append(math.degrees(x))
            self.plot_data["encoder_position_plot"][1].append(math.degrees(y))

    def record_motors(self, values):
        if self.plot_enabled["plot_calculated_state"]:
            self.filter.update_motors(values)

    def record_servo(self, values):
        if self.plot_enabled["plot_calculated_state"]:
            self.filter.update_servo(values)


def parse_arguments():
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
        file_name = -1
        directory = "Jul 29 2016"
        map_name = "test goal track"
        map_dir = None

    try:
        file_name = int(file_name)
    except ValueError:
        pass

    return file_name, directory, map_name, map_dir


def run():
    file_name, directory, map_name, map_dir = parse_arguments()

    plotter = FilterTest(
        file_name, directory, map_name, map_dir, RcCarFilter,
        map_plot=dict(color='fuchsia', label="map", log_based_plot=False),
        checkpoints_plot=dict(color='o', label="checkpoints",
                              log_based_plot=False, markersize=4),

        gps_plot=dict(color='r', label="GPS"),

        # waypoints_plot=dict(color='purple', line_segments=True,
        #                     line_seg_freq=100),

        calculated_state_plot=dict(color='g', label="state"),
        # heading_lines_plot=dict(line_segments=True, color='orange',
        #                         line_seg_freq=50),

        # recorded_state_plot=dict(color='b', label="recorded state"),
        # recorded_heading_lines_plot=dict(line_segments=True, color='red',
        #                                  line_seg_freq=50),

        checkpoint_lines_plot=dict(line_segments=True, color='darkgreen'),

        # encoder_position_plot=dict(color='r', label="encoder position"),
    )

    plotter.run()


run()
