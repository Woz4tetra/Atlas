
import sys
import math

from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection
# from atlasbuggy.files.mapfile import MapFile
from algorithms.kalman_filter import GrovesKalmanFilter, get_gps_orientation
from algorithms.kalman_constants import constants

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

# import numpy as np

animate = True
use_filter = True


class Simulator(RobotInterfaceSimulator):
    def __init__(self, set_num=0):
        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret(angle_range=(210, 330), reverse_range=True)

        self.imu_plot = RobotPlot("Magnetometer data (x, z vs. time)", flat_plot=True, skip_count=20,
                                  plot_enabled=False)
        self.gps_plot = RobotPlot(
            "GPS data", flat_plot=False, color='red', skip_count=0,
            # marker='.', linestyle='',
            # plot_enabled=False
        )
        self.kalman_plot = RobotPlot(
            "Kalman filter post", flat_plot=False, skip_count=0,
            # linestyle='', color='blue', marker='.',
            # plot_enabled=False
        )

        self.compass_plot = RobotPlot("compass", color='orange')
        self.filter_comparison = RobotPlotCollection("Comparison", self.kalman_plot, self.gps_plot, self.compass_plot,
                                                     flat_plot=True)
        # self.map_plot = RobotPlot("map data")
        # self.turret.point_cloud_plot.skip_count = 10

        if animate:
            self.live_plot = LivePlotter(
                2,
                self.filter_comparison,
                self.turret.point_cloud_plot,
                plot_skip_count=10)
        else:
            self.compass_plot.enabled = False
            self.static_plot = StaticPlotter(2, self.imu_plot, self.filter_comparison)

        self.filter = None

        # m = np.array(MapFile("buggy course checkpoints").map)
        # self.map_plot.update(m[:, 0], m[:, 1])

        data_day_3 = "data_days/2017_02_08"
        old_data = "old_data/"
        file_sets = (
            ("16;38", data_day_3),
            ("17", data_day_3),
            ("18", data_day_3),
            (None, None),

            # trackfield no gyro values
            # ("15;46", old_data + "2016_Dec_02"),
            # ("15;54", old_data + "2016_Dec_02"),
            # ("16;10", old_data + "2016_Dec_02"),
            # ("16;10", old_data + "2016_Dec_02")

            # rolls day 1
            # ("07;22", old_data + "2016_Nov_06"),  # bad gyro values

            # rolls day 2
            # ("07;36;03 m", old_data + "2016_Nov_12"),
            # ("09;12", old_data + "2016_Nov_12"),  # invalid values
            # ("07;04;57", old_data + "2016_Nov_13"),  # bad gyro values

            # rolls day 3
            # ("modified 07;04", old_data + "2016_Nov_13"),
            # ("modified 07;23", old_data + "2016_Nov_13"),  # wonky value for mag.

            # rolling on the cut
            # ("16;29", old_data + "2016_Dec_09"),
            # ("16;49", old_data + "2016_Dec_09"),
            # ("16;5", old_data + "2016_Dec_09"),
            # ("17;", old_data + "2016_Dec_09"),  # nothing wrong, really short

            # bad data
            # ("16;07", old_data + "2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("16;09", old_data + "2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("18;00", old_data + "2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;02", old_data + "2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;09", old_data + "2016_Dec_09/bad_data"),  # gps spazzed out
        )

        self.lat1, self.long1, self.alt1 = 0, 0, 0
        self.lat2, self.long2, self.alt2 = 0, 0, 0
        self.num_recv_from_fix = None
        self.received_fix = False

        self.imu_t0 = 0
        self.gps_t0 = 0

        super(Simulator, self).__init__(
            file_sets[set_num][0], file_sets[set_num][1],
            self.gps, self.imu,
            self.turret,
            # start_index=0,
            # end_index=2000,
        )
        print("Using:", self.parser.full_path)

    def init_filter(self, timestamp):
        self.lat2, self.long2, self.alt2 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude
        yaw, pitch, roll = get_gps_orientation(self.lat1, self.long1, self.alt1,
                                               self.lat2, self.long2, self.alt2)

        self.filter = GrovesKalmanFilter(
            initial_roll=roll,
            initial_pitch=pitch,
            initial_yaw=yaw,
            initial_lat=self.lat2,
            initial_long=self.long2,
            initial_alt=self.alt2,
            **constants
        )

        self.imu_t0 = timestamp
        self.gps_t0 = timestamp

        self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
        self.kalman_plot.append(*self.filter.get_position())

    def update_gps_plot(self):
        if not 280 < self.gps.altitude < 310:
            self.gps.altitude = 300.0
        if -80 < self.gps.longitude_deg < -79.8 and 40.4 < self.gps.latitude_deg < 45 and (
                            280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
            self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
            return True
        else:
            return False

    def update_epoch(self, timestamp):
        self.filter.gps_updated(timestamp - self.gps_t0,
                                self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
        self.kalman_plot.append(*self.filter.get_position())
        self.gps_t0 = timestamp

        lat1, long1, alt1 = self.filter.get_position()
        roll, pitch, yaw = self.filter.get_orientation()

        # length = math.sqrt(lat1 ** 2 + long1 ** 2)
        lat2 = 0.0003 * math.sin(yaw) + lat1
        long2 = 0.0003 * math.cos(yaw) + long1

        self.compass_plot.update([lat1, lat2], [long1, long2])

    def update_ins(self, timestamp):
        if self.filter is not None and self.imu_t0 != timestamp:
            self.filter.imu_updated(timestamp - self.imu_t0,
                                    self.imu.linaccel.x, self.imu.linaccel.y, self.imu.linaccel.z,
                                    self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z)
            self.kalman_plot.append(*self.filter.get_position())

            self.imu_t0 = timestamp

    def object_packet(self, timestamp):
        if self.did_receive(self.imu):
            # print("imu", timestamp)
            self.imu_plot.append(timestamp, self.imu.accel.z, self.imu.accel.x)

            if use_filter:
                self.update_ins(timestamp)

        elif self.did_receive(self.gps):
            if self.gps.fix and self.gps.latitude is not None and self.received_fix != self.gps.fix:
                self.received_fix = True
                self.num_recv_from_fix = self.num_received(self.gps)

            if self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix == 2:
                self.lat1, self.long1, self.alt1 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude

            elif self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix == 150:
                if use_filter:
                    self.init_filter(timestamp)

            elif self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix > 150:
                # if self.num_received(self.gps) % 5 == 0:
                if self.update_gps_plot():
                    if use_filter:
                        if self.update_epoch(timestamp) is False:
                            return False

                    if animate and self.live_plot.should_update():
                        if self.live_plot.plot() is False:
                            return False
                else:
                    print(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
                    if use_filter:
                        print(self.filter.get_position(), self.filter.properties.estimated_velocity.T.tolist())
        elif self.did_receive(self.turret):
            if self.turret.did_cloud_update():
                # if self.turret.is_cloud_ready():

                if not self.live_plot.plot():
                    return False

    def close(self):
        if not animate:
            self.static_plot.plot()
            self.static_plot.show()
        else:
            if not self.error_signalled:
                print("Finished!")
                self.live_plot.freeze_plot()

            self.live_plot.close()


if len(sys.argv) >= 2:
    Simulator(int(sys.argv[1])).run()
else:
    Simulator(-1).run()
