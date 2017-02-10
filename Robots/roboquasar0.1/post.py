import sys

from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.robotplot import RobotPlot
from atlasbuggy.files.mapfile import MapFile

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

        self.imu_plot = RobotPlot("Magnetometer data (x, z vs. time)", flat_plot=False, skip_count=20)
        self.gps_plot = RobotPlot("GPS data", flat_plot=False)
        # self.map_plot = RobotPlot("map data")

        self.staticPlot = StaticPlotter(2, self.imu_plot, self.gps_plot)

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
        elif self.did_receive(self.gps) and self.gps.fix is True and self.gps.latitude is not None:
            if self.gps.longitude_deg < 0:
                self.gps.longitude_deg *= -1
            if 79.8 < self.gps.longitude_deg < 80 and 40.4 < self.gps.latitude_deg < 45 and (
                        280 < self.gps.altitude < 310 or self.gps.altitude == 0.0):
                self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)
            else:
                print(self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude)

    def close(self):
        self.staticPlot.plot()
        self.staticPlot.show()


if len(sys.argv) >= 2:
    Simulator(int(sys.argv[1])).run()
else:
    Simulator(1).run()
