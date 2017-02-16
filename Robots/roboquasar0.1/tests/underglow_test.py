import time
import os

os.chdir("..")

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.robot.simulator import RobotInterfaceSimulator

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.steering import Steering
from actuators.brakes import Brakes
from actuators.underglow import Underglow

simulated = True


class UnderglowSimulator(RobotInterfaceSimulator):
    def __init__(self, file_set_num):
        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow(enabled=False)  # so that packets don't get queued for no reason

        self.plotter = LivePlotter(1, self.underglow.strip_plot, enable_legend=False)

        for index, led in enumerate(self.underglow.led_plots):
            led.update([index], [0])

        self.counter = 0

        file_sets = (
            (None, None),  # default, latest

            # data day 4
            # ("15", "data_days/2017_Feb_14"),  # filter explodes, LIDAR interfered by the sun
            # ("16;20", "data_days/2017_Feb_14"),  # shorten run, LIDAR collapsed
            # ("16;57", "data_days/2017_Feb_14"),  # interfered LIDAR, filter explodes at the beginning, GPS jumps
            # ("17;10", "data_days/2017_Feb_14"),  # beginning: all data is fine, filter jumps A LOT; middle: filter explodes, interfered LIDAR
            ("17;33", "data_days/2017_Feb_14"),  # data is fine, normal run

            # data day 3
            # ("16;38", "data_days/2017_Feb_08"),
            # ("17", "data_days/2017_Feb_08"),
            # ("18", "data_days/2017_Feb_08"),

            # trackfield no gyro values
            # ("15;46", "old_data/2016_Dec_02"),
            # ("15;54", "old_data/2016_Dec_02"),
            # ("16;10", "old_data/2016_Dec_02"),
            # ("16;10", "old_data/2016_Dec_02")

            # rolls day 1
            # ("07;22", "old_data/2016_Nov_06"),  # bad gyro values

            # rolls day 2
            # ("07;36;03 m", "old_data/2016_Nov_12"),
            # ("09;12", "old_data/2016_Nov_12"),  # invalid values
            # ("07;04;57", "old_data/2016_Nov_13"),  # bad gyro values

            # rolls day 3
            # ("modified 07;04", "old_data/2016_Nov_13"),
            # ("modified 07;23", "old_data/2016_Nov_13"),  # wonky value for mag.

            # rolling on the cut
            # ("16;29", "old_data/2016_Dec_09"),
            # ("16;49", "old_data/2016_Dec_09"),
            # ("16;5", "old_data/2016_Dec_09"),
            # ("17;", "old_data/2016_Dec_09"),  # nothing wrong, really short

            # bad data
            # ("16;07", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("16;09", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
            # ("18;00", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;02", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
            # ("18;09", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        )

        super(UnderglowSimulator, self).__init__(
            file_sets[file_set_num][0], file_sets[file_set_num][1],
            self.gps, self.imu, self.turret, self.brakes, self.steering, self.underglow
            # start_index=0,
            # end_index=2000,
        )

    def object_packet(self, timestamp):
        if self.did_receive(self.imu):
            pass
        elif self.did_receive(self.gps):
            pass
        elif self.did_receive(self.turret):
            pass

    def loop(self):
        for num in range(self.underglow.num_leds):
            self.underglow.set_led(num, self.counter, self.counter, self.counter)

        self.counter += 1
        if self.counter > 255:
            self.counter = 0

        if self.plotter.plot() is False:
            return False

    def close(self):
        self.plotter.close()


UnderglowSimulator(-1).run()
