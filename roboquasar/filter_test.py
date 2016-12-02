import numpy as np
from autobuggy import project

from autobuggy.simulator import Simulator
from autobuggy.microcontroller.logger import get_map, parse_arguments
from autobuggy.filters.kalman_filter import GrovesKalmanFilter, \
    get_gps_orientation
from roboquasar_constants import constants

disable_ins = False
disable_epoch = False

if disable_epoch:
    print("EPOCH is disabled!")

if disable_ins:
    print("INS is disabled!")

project.set_project_dir("roboquasar")


class FilterTest(Simulator):
    def __init__(self, file_name, directory, enable_3d, **plot_info):
        self.checkpoints = get_map("buggy course checkpoints.gpx")
        self.course_map = get_map("buggy course map.gpx")

        super(FilterTest, self).__init__(
            file_name, directory, 1, plot_info, enable_3d,  # 0, 5000
        )

        if enable_3d:
            self.plot_info["calculated_filter_plot"]["skip_count"] = 1000

        first_gps = self.parser.get(5, "gps")[-1]
        second_gps = self.parser.get(50, "gps")[-1]

        lat1, long1, alt1 = first_gps["lat"], first_gps["long"], first_gps[
            "altitude"]
        lat2, long2, alt2 = second_gps["lat"], second_gps["long"], second_gps[
            "altitude"]

        initial_yaw, initial_pitch, initial_roll = \
            get_gps_orientation(lat1, long1, alt1,
                                lat2, long2, alt2)

        self.filter = GrovesKalmanFilter(
            initial_roll=initial_roll,
            initial_pitch=initial_pitch,
            initial_yaw=initial_yaw,
            initial_lat=lat1,
            initial_long=long1,
            initial_alt=alt1,
            **constants)

        self.draw_dot(long1, lat1, alt1)

        self.prev_lat, self.prev_long = lat1, long1

        self.prev_imu_t = self.start_time
        self.prev_gps_t = self.start_time

    def fill_data_array(self, plot_option, data_array):
        if plot_option == "checkpoints_plot":
            checkpoints_map = np.array(self.checkpoints)
            data_array[0] = checkpoints_map[:, 1]
            data_array[1] = checkpoints_map[:, 0]
            if self.enable_3d:
                data_array[2] = 300
        elif plot_option == "course_map_plot":
            course_map = np.array(self.course_map)
            data_array[0] = course_map[:, 1]
            data_array[1] = course_map[:, 0]
            if self.enable_3d:
                data_array[2] = 300

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.record_imu(timestamp, values)
        elif name == "gps":
            self.record_gps(timestamp, values)

    def record_imu(self, timestamp, values):
        if self.plot_enabled["imu_plot"]:
            self.angled_line_segment("imu_plot", self.prev_long, self.prev_lat,
                                     values['yaw'], 1)
        if self.plot_enabled["calculated_filter_plot"] and not disable_ins:
            self.filter.imu_updated(
                timestamp - self.prev_imu_t,
                values["ax"], values["ay"], values["az"],
                values["gx"], values["gy"], values["gz"],
            )
            self.prev_imu_t = timestamp

            self.record_position()

    def record_gps(self, timestamp, values):
        if self.plot_enabled["gps_plot"]:
            if values["long"] != self.prev_long or values[
                "lat"] != self.prev_lat:
                self.plot_data["gps_plot"][0].append(values["long"])
                self.plot_data["gps_plot"][1].append(values["lat"])
                if self.enable_3d:
                    self.plot_data["gps_plot"][2].append(values["altitude"])

                self.prev_lat = values["lat"]
                self.prev_long = values["long"]

                if self.plot_enabled[
                    "calculated_filter_plot"] and not disable_epoch:
                    self.filter.gps_updated(
                        timestamp - self.prev_gps_t,
                        values["lat"], values["long"], values["altitude"]
                    )
                    self.prev_gps_t = timestamp

                    # lat, long, height = self.filter.get_position()
                    # self.draw_dot(long, lat, 'green')

                    self.record_position()

    def record_position(self):
        lat, long, height = self.filter.get_position()
        self.plot_data["calculated_filter_plot"][0].append(long)
        self.plot_data["calculated_filter_plot"][1].append(lat)
        if self.enable_3d:
            self.plot_data["calculated_filter_plot"][2].append(height)

        yaw, pitch, roll = self.filter.get_orientation()

        if self.plot_enabled["calculated_filter_heading_plot"]:
            self.angled_line_segment("calculated_filter_heading_plot", long,
                                     lat, yaw, 1)


class GraphAccel(Simulator):
    def __init__(self, file_name, directory, enable_3d, **plot_info):
        self.checkpoints = get_map("buggy course checkpoints.gpx")
        self.course_map = get_map("buggy course map.gpx")

        super(GraphAccel, self).__init__(
            file_name, directory, 1, plot_info, enable_3d,
        )

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.plot_data["accel_plot_x"][0].append(timestamp)
            self.plot_data["accel_plot_x"][1].append(values["ax"])

            self.plot_data["accel_plot_y"][0].append(timestamp)
            self.plot_data["accel_plot_y"][1].append(values["ay"])

            self.plot_data["accel_plot_z"][0].append(timestamp)
            self.plot_data["accel_plot_z"][1].append(values["az"])


def run():
    file_name, directory = parse_arguments(-1, -2)

    plotter = FilterTest(
        file_name, directory,
        # imu_plot=dict(color='orange', line_segments=True, line_seg_freq=50),
        gps_plot=dict(color='red', label="GPS"),
        calculated_filter_plot=dict(color='fuchsia', label="filter"),
        calculated_filter_heading_plot=dict(color='purple', line_segments=True,
                                            line_seg_freq=50),
        # checkpoints_plot=dict(color='green', label="checkpoints",
        #                       log_based_plot=False),
        course_map_plot=dict(color='blue', label="map",
                             log_based_plot=False),
        enable_3d=True
    )

    # plotter = GraphAccel(file_name, directory, False,
    #                      accel_plot_x=dict(color='red', alpha=0.5),
    #                      accel_plot_y=dict(color='green', alpha=0.5),
    #                      accel_plot_z=dict(color='blue', alpha=0.5),
    #                      )

    plotter.run()


run()
