import sys
import math
import numpy as np
from autobuggy import project

from autobuggy.simulator import Simulator
from autobuggy.microcontroller.logger import get_map
from autobuggy.filters.kalman_filter import GrovesKalmanFilter, get_gps_orientation
from roboquasar_constants import constants


class FilterTest(Simulator):
    def __init__(self, file_name, directory, **plot_info):
        project.set_project_dir("roboquasar")
        self.checkpoints = get_map("buggy course checkpoints.gpx")
        self.course_map = get_map("buggy course map.gpx")

        super(FilterTest, self).__init__(
            file_name, directory, 1, plot_info, 0, 5000
        )
        
        first_gps = self.parser.get(5, "gps")[-1]
        second_gps = self.parser.get(50, "gps")[-1]
        
        lat1, long1, alt1 = first_gps["lat"], first_gps["long"], first_gps["altitude"] 
        lat2, long2, alt2 = second_gps["lat"], second_gps["long"], second_gps["altitude"]
        
        print(lat1, long1, alt1)
        print(lat2, long2, alt2)
        
        initial_yaw, initial_pitch, initial_roll = \
            get_gps_orientation(lat1, long1, alt1,
                                lat2, long2, alt2)

        # print(np.array([initial_yaw, initial_pitch, initial_roll]) * 180 / np.pi)
        self.filter = GrovesKalmanFilter(
            initial_roll=initial_roll,
            initial_pitch=initial_pitch,
            initial_yaw=initial_yaw,
            initial_lat=lat1,
            initial_long=long1,
            initial_alt=alt1,
            **constants)

        self.draw_dot(long1, lat1)

        self.prev_lat, self.prev_long = lat1, long1

        self.prev_imu_t = self.start_time
        self.prev_gps_t = self.start_time

        self.filter_returned_value = False

    def fill_data_array(self, plot_option, data_array):
        if plot_option == "checkpoints_plot":
            checkpoints_map = np.array(self.checkpoints)
            data_array[0] = checkpoints_map[:, 1]
            data_array[1] = checkpoints_map[:, 0]
        elif plot_option == "course_map_plot":
            course_map = np.array(self.course_map)
            data_array[0] = course_map[:, 1]
            data_array[1] = course_map[:, 0]

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.record_imu(timestamp, values)
        elif name == "gps":
            self.record_gps(timestamp, values)

    def record_imu(self, timestamp, values):
        if self.plot_enabled["imu_plot"]:
            self.angled_line_segment("imu_plot", self.prev_long, self.prev_lat,
                                     values['yaw'], 0.5)
        if self.plot_enabled["calculated_filter_plot"]:
            self.filter.imu_updated(
                timestamp - self.prev_imu_t,
                values["ax"], values["ay"], values["az"],
                values["gx"], values["gy"], values["gz"],
            )
            self.prev_imu_t = timestamp

            self.record_position()

    def record_gps(self, timestamp, values):
        if self.plot_enabled["gps_plot"]:
            if values["long"] != self.prev_long or values["lat"] != self.prev_lat:
                self.plot_data["gps_plot"][0].append(values["long"])
                self.plot_data["gps_plot"][1].append(values["lat"])

                self.prev_lat = values["lat"]
                self.prev_long = values["long"]

                # if self.plot_enabled["calculated_filter_plot"]:
                #     self.filter.gps_updated(
                #         timestamp - self.prev_gps_t,
                #         values["lat"], values["long"], values["altitude"]
                #     )
                #     self.prev_gps_t = timestamp
                #
                #     # lat, long, height = self.filter.get_position()
                #     # self.draw_dot(long, lat, 'green')
                #
                #     self.record_position()

    def record_position(self):
        lat, long, height = self.filter.get_position()
        self.plot_data["calculated_filter_plot"][0].append(long)
        self.plot_data["calculated_filter_plot"][1].append(lat)
        
        yaw, pitch, roll = self.filter.get_orientation()

        if not self.filter_returned_value:
            self.draw_dot(long, lat, "red")

            self.filter_returned_value = True
            print("filter first value:", lat, long)
        
        if self.plot_enabled["calculated_filter_heading_plot"]:
            self.angled_line_segment("calculated_filter_heading_plot", long, lat, yaw, 0.5)


def parse_arguments():
    file_name = -1
    directory = -1

    if len(sys.argv) == 2:
        file_name = sys.argv[1]
    elif len(sys.argv) == 3:
        file_name, directory = sys.argv[1:]

    try:
        file_name = int(file_name)
        directory = int(directory)
    except ValueError:
        pass

    return file_name, directory

def run():
    file_name, directory = parse_arguments()
    print(file_name, directory)

    plotter = FilterTest(
        file_name, directory,
#        imu_plot=dict(color='orange', line_segments=True, line_seg_freq=50),
        gps_plot=dict(color='red', label="GPS"),
        calculated_filter_plot=dict(color='fuchsia', label="filter"),
        calculated_filter_heading_plot=dict(color='purple', line_segments=True, line_seg_freq=50),
        # checkpoints_plot=dict(color='green', label="checkpoints",
        #                       log_based_plot=False),
        # course_map_plot=dict(color='blue', label="map",
        #                      log_based_plot=False)
    )

    plotter.run()


run()
