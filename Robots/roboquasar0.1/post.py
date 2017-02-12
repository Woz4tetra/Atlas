import sys

from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection
# from atlasbuggy.files.mapfile import MapFile
from algorithms.kalman_filter import GrovesKalmanFilter, get_gps_orientation
from algorithms.kalman_constants import constants

from sensors.gps import GPS
from sensors.imu import IMU
from actuators.steering import Steering
from actuators.brakes import Brakes

import numpy as np


class Simulator(RobotInterfaceSimulator):
    def __init__(self, set_num=0):
        self.gps = GPS()
        self.imu = IMU()
        self.steering = Steering()
        self.brakes = Brakes()

        self.imu_plot = RobotPlot("Magnetometer data (x, z vs. time)", flat_plot=True, skip_count=20)
        self.gps_plot = RobotPlot("GPS data", flat_plot=False, color='red', plot_enabled=False)
        self.kalman_plot = RobotPlot("Kalman filter post", flat_plot=False, linewidth=2, color='orange')

        self.filter_comparison = RobotPlotCollection("Comparison", self.kalman_plot, self.gps_plot)
        # self.map_plot = RobotPlot("map data")

        self.staticPlot = StaticPlotter(2, self.imu_plot, self.filter_comparison)

        self.filter = None

        # m = np.array(MapFile("buggy course checkpoints").map)
        # self.map_plot.update(m[:, 0], m[:, 1])

        data_day_3 = "data_days/2017_02_08"
        file_sets = (
            (None, None),
            ("16;38", data_day_3),
            ("17", data_day_3),
            ("18", data_day_3),
            ("22", data_day_3),
        )

        self.lat1, self.long1, self.alt1 = 0, 0, 0
        self.lat2, self.long2, self.alt2 = 0, 0, 0
        self.num_recv_from_fix = None
        self.received_fix = False

        self.imu_t0 = 0
        self.gps_t0 = 0

        super(Simulator, self).__init__(
            file_sets[set_num][0], file_sets[set_num][1],
            self.gps, self.imu, self.steering, self.brakes,
            # start_index=1000,
            # end_index=2000,
        )
        print("Using:", self.parser.full_path)

    def object_packet(self, timestamp):
        if self.did_receive(self.imu):
            self.imu_plot.append(timestamp, self.imu.accel.z, self.imu.accel.x)

            if self.filter is not None and self.imu_t0 != timestamp:
                self.filter.imu_updated(timestamp - self.imu_t0,
                                        self.imu.accel.x, self.imu.accel.y, self.imu.accel.z,
                                        self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z)
                # self.kalman_plot.append(*self.filter.get_position())

            self.imu_t0 = timestamp

        elif self.did_receive(self.gps):
            if self.gps.fix and self.gps.latitude is not None and self.received_fix != self.gps.fix:
                self.received_fix = True
                self.num_recv_from_fix = self.num_received(self.gps)

            if self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix == 2:
                self.lat1, self.long1, self.alt1 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude

            elif self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix == 4:
                self.lat2, self.long2, self.alt2 = self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude
                yaw, pitch, roll = get_gps_orientation(self.lat1, self.long1, self.alt1,
                                                       self.lat2, self.long2, self.alt2)
                self.filter = GrovesKalmanFilter(
                    initial_roll=roll,
                    initial_pitch=pitch,
                    initial_yaw=yaw,
                    initial_lat=self.lat1,
                    initial_long=self.long1,
                    initial_alt=self.alt1,
                    **constants
                )
                self.imu_t0 = timestamp
                self.gps_t0 = timestamp

            elif self.num_recv_from_fix is not None and self.num_received(self.gps) - self.num_recv_from_fix > 4:
                if self.gps.longitude_deg < 0:
                    self.gps.longitude_deg *= -1
                if 79.8 < self.gps.longitude_deg < 80 and 40.4 < self.gps.latitude_deg < 45 and (
                                    280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
                    self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)

                    self.filter.gps_updated(timestamp - self.gps_t0,
                                            self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
                    self.kalman_plot.append(*self.filter.get_position())
                    self.gps_t0 = timestamp
                else:
                    print(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)

    def close(self):
        self.staticPlot.plot()
        self.staticPlot.show()


if len(sys.argv) >= 2:
    Simulator(int(sys.argv[1])).run()
else:
    Simulator(2).run()
