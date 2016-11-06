import sys
import math
import numpy as np
from autobuggy import project

from autobuggy.simulator import Simulator
from autobuggy.microcontroller.logger import get_map


class FilterTest(Simulator):
    def __init__(self, file_name, directory, **plot_info):
        project.set_project_dir("roboquasar")
        self.checkpoints = get_map("checkpoints.txt")

        super(FilterTest, self).__init__(
            file_name, directory, 1, plot_info
        )
        
        first_gps_reading = self.parser.get('gps')[-1]
        initial_lat = first_gps_reading["lat"]
        initial_long = first_gps_reading["long"]
        self.draw_starting_dot(initial_lat, initial_long)
        
        self.prev_lat, self.prev_long = initial_lat, initial_long

    def fill_data_array(self, plot_option, data_array):
        if plot_option == "checkpoints_plot":
            checkpoints_map = np.array(self.checkpoints)
            data_array[0] = checkpoints_map[:, 0]
            data_array[1] = checkpoints_map[:, 1]

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.record_imu(timestamp, values)
        elif name == "gps":
            self.record_gps(timestamp, values)
        elif name == "checkpoint":
            self.record_checkpoint(values)

    def record_imu(self, timestamp, values):
        if self.plot_enabled["imu_plot"]:
            self.angled_line_segment("imu_plot", self.prev_lat, self.prev_long,
                values['yaw'], 0.5)

    def record_gps(self, timestamp, values):
        if self.plot_enabled["gps_plot"]:
            self.plot_data["gps_plot"][0].append(values["lat"])
            self.plot_data["gps_plot"][1].append(values["long"])
            
            self.prev_lat = values["lat"]
            self.prev_long = values["long"]


def parse_arguments():
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name, directory = sys.argv[1:]
#    elif len(sys.argv) == 4:
#        file_name, directory, map_name = sys.argv[1:]
#        map_dir = ":gpx"
#    elif len(sys.argv) == 5:
#        file_name, directory, map_name, map_dir = sys.argv[1:]

    else:
        file_name = -1
        directory = "Nov 05 2016"

    try:
        file_name = int(file_name)
    except ValueError:
        pass

    return file_name, directory


def run():
    file_name, directory = parse_arguments()

    plotter = FilterTest(
        file_name, directory,
#        map_plot=dict(color='fuchsia', label="map", log_based_plot=False),
#        checkpoints_plot=dict(color='o', label="checkpoints",
#                              log_based_plot=False, markersize=4),
        imu_plot=dict(color='orange', line_segments=True, line_seg_freq=50),
        gps_plot=dict(color='red', label="GPS"),
    )

    plotter.run()


run()
