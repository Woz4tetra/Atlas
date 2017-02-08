import numpy as np
from atlasbuggy import project

from atlasbuggy.simulator import Simulator
from atlasbuggy.microcontroller.logger import get_map, parse_arguments
from atlasbuggy.filters.kalman_filter import GrovesKalmanFilter, \
    get_gps_orientation
from roboquasar_constants import constants

project.set_project_dir("roboquasar0.0")


class FilterTest(Simulator):
    def __init__(self, file_name, directory, enable_3d, use_pickled_data=False,
                 start_index=0,
                 end_index=-1, **plot_info):
        self.checkpoints = get_map(
            # "buggy_course/buggy course checkpoints.gpx"
            "track_field/track field checkpoints.gpx"
            # "cut/cut course checkpoints.gpx"
         )
        self.course_map = get_map(
            # "buggy_course/buggy course map.gpx"
            "track_field/track field course map.gpx"
            # "cut/cut course map 2.gpx"
        )

        super(FilterTest, self).__init__(
            file_name, directory, 1, plot_info, enable_3d, use_pickled_data,
            start_index, end_index
        )

        if enable_3d:
            self.plot_info["gps_plot"]["skip_count"] = 5
            # self.plot_info["calculated_filter_plot"]["skip_count"] = 200
            # self.plot_info["recorded_filter_plot"]["skip_count"] = 200

        if self.plot_enabled["recorded_filter_plot"]:
            first_gps = self.parser.get(5, "gps")[-1]
            second_gps = self.parser.get(50, "gps")[-1]
            self.lat1, self.long1, self.alt1 = \
                first_gps["lat"], first_gps["long"], first_gps["altitude"]
            self.lat2, self.long2, self.alt2 = \
                second_gps["lat"], second_gps["long"], second_gps["altitude"]

            self.initial_yaw, self.initial_pitch, self.initial_roll = \
                get_gps_orientation(self.lat1, self.long1, self.alt1,
                                    self.lat2, self.long2, self.alt2)
        else:
            initial_state = self.parser.get(0, "kalman")[-1]

            self.lat1 = initial_state["lat"]
            self.long1 = initial_state["long"]
            self.alt1 = initial_state["alt"]
            self.initial_yaw = initial_state["yaw"]
            self.initial_pitch = initial_state["pitch"]
            self.initial_roll = initial_state["roll"]

        self.filter = GrovesKalmanFilter(
            initial_roll=self.initial_roll,
            initial_pitch=self.initial_pitch,
            initial_yaw=self.initial_yaw,
            initial_lat=self.lat1,
            initial_long=self.long1,
            initial_alt=self.alt1,
            **constants
        )

        self.draw_dot(self.long1, self.lat1, self.alt1)

        self.prev_lat, self.prev_long = self.lat1, self.long1

        self.prev_imu_t = self.start_time
        self.prev_gps_t = self.start_time

    def fill_data_array(self, plot_option, data_array):
        if plot_option == "checkpoints_plot":
            checkpoints_map = np.array(self.checkpoints)
            data_array[0] = checkpoints_map[:, 1]
            data_array[1] = checkpoints_map[:, 0]
            if self.enable_3d:
                data_array[2] = self.alt1
        elif plot_option == "course_map_plot":
            course_map = np.array(self.course_map)
            data_array[0] = course_map[:, 1]
            data_array[1] = course_map[:, 0]
            if self.enable_3d:
                data_array[2] = self.alt1 - 5

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.record_imu(timestamp, values)
        elif name == "gps":
            self.record_gps(timestamp, values)
        elif name == "kalman":
            self.record_kalman_data(values)

    def record_kalman_data(self, values):
        if self.plot_enabled["recorded_filter_plot"]:
            self.plot_data["recorded_filter_plot"][0].append(values["long"])
            self.plot_data["recorded_filter_plot"][1].append(values["lat"])
            if self.enable_3d:
                self.plot_data["recorded_filter_plot"][2].append(values["alt"])
        if self.plot_enabled["recorded_filter_heading_plot"]:
            self.angled_line_segment(
                "recorded_filter_heading_plot",
                values["long"], values["lat"], values['yaw'], 1)

    def record_imu(self, timestamp, values):
        if self.plot_enabled["imu_plot"]:
            self.angled_line_segment("imu_plot", self.prev_long, self.prev_lat,
                                     values['yaw'], 1)
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
            if values["long"] != self.prev_long or \
                            values["lat"] != self.prev_lat:
                self.plot_data["gps_plot"][0].append(values["long"])
                self.plot_data["gps_plot"][1].append(values["lat"])
                if self.enable_3d:
                    self.plot_data["gps_plot"][2].append(values["altitude"])

                self.prev_lat = values["lat"]
                self.prev_long = values["long"]

                if self.plot_enabled["calculated_filter_plot"]:
                    self.filter.gps_updated(
                        timestamp - self.prev_gps_t,
                        values["lat"], values["long"], values["altitude"]
                    )
                    self.prev_gps_t = timestamp

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

    def before_plot(self):
        if self.enable_3d:
            plot_option_bounds = ""
            for option in ["gps_plot", "recorded_filter_plot",
                           "calculated_filter_plot"]:
                if option in self.plot_data.keys():
                    plot_option_bounds = option
                    break

            if plot_option_bounds != "":
                x0 = min(self.plot_data[plot_option_bounds][0])
                x1 = max(self.plot_data[plot_option_bounds][0])

                y0 = min(self.plot_data[plot_option_bounds][1])
                y1 = max(self.plot_data[plot_option_bounds][1])

                if self.enable_3d:
                    z0 = min(self.plot_data[plot_option_bounds][2])
                    z1 = max(self.plot_data[plot_option_bounds][2])
                    self.graph_limits(x0, x1, y0, y1, z0, z1)
                else:
                    self.graph_limits(x0, x1, y0, y1)


class GraphSensor(Simulator):
    def __init__(self, file_name, directory, enable_3d, **plot_info):
        self.checkpoints = get_map("buggy course checkpoints.gpx")
        self.course_map = get_map("buggy course map.gpx")

        super(GraphSensor, self).__init__(
            file_name, directory, 1, plot_info, enable_3d,
        )

    def step(self, index, timestamp, name, values):
        if name == "imu":
            self.plot_data["plot_1"][0].append(timestamp)
            self.plot_data["plot_1"][1].append(values["ax"])

            self.plot_data["plot_2"][0].append(timestamp)
            self.plot_data["plot_2"][1].append(values["ay"])

            self.plot_data["plot_3"][0].append(timestamp)
            self.plot_data["plot_3"][1].append(values["az"])


file_name, directory = parse_arguments(-1, "Dec 02 2016")


def run_kalman():
    plotter = FilterTest(
        file_name, directory,
        # imu_plot=dict(color='orange', line_segments=True, line_seg_freq=50),
        gps_plot=dict(color='lightskyblue', label="GPS"),
        calculated_filter_plot=dict(color='indigo', label="filter"),
        # calculated_filter_heading_plot=dict(color='lime', line_segments=True,
        #                                     line_seg_freq=50),
        recorded_filter_plot=dict(color='forestgreen', label="recorded filter"),
        # recorded_filter_heading_plot=dict(color='teal',
        #                                   line_segments=True,
        #                                   line_seg_freq=50),
        # checkpoints_plot=dict(color='deepskyblue', label="checkpoints",
        #                       log_based_plot=False),
        course_map_plot=dict(color='gold', label="map",
                             log_based_plot=False),
        enable_3d=False,
        use_pickled_data=True,
        start_index=0,
        # end_index=5000
    )

    plotter.run()


def run_sensor():
    plotter = GraphSensor(
        file_name, directory, False,
        plot_1=dict(color='red', alpha=0.5),
        plot_2=dict(color='green', alpha=0.5),
        plot_3=dict(color='blue', alpha=0.5),
    )

    plotter.run()


run_kalman()
# run_sensor()
