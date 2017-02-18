# Compare checkpoint data to checkpoint maps
import math

from atlasbuggy.interface.simulated import SimulatedRobot

from sensors.gps import GPS
from sensors.imu import IMU

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from atlasbuggy.files.mapfile import MapFile

from data_sets import *


class Animator(SimulatedRobot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.checkpoints = MapFile("02-17-2017 Test Run1")

        self.gps_plot = RobotPlot("gps")
        self.checkpoint_plot = RobotPlot("checkpoint", marker='.', linestyle='', markersize=8)
        self.map_plot = RobotPlot("map")
        self.compass_plot = RobotPlot("compass")

        self.map_plot.update(self.checkpoints.lats, self.checkpoints.longs)

        self.accuracy_check_plot = RobotPlotCollection("Animation", self.gps_plot, self.checkpoint_plot,
                                                       self.map_plot, self.compass_plot)

        self.plotter = LivePlotter(1, self.accuracy_check_plot)

        file_name, directory = file_sets["data day 5"][-1]
        super(Animator, self).__init__(
            file_name, directory,
            self.gps, self.imu, self.steering, self.brakes, self.underglow
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.gps.is_position_valid():
            if self.did_receive("checkpoint"):
                self.checkpoint_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(self.gps):
                self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(self.imu):
                lat2 = 0.0003 * math.sin(-self.imu.euler.z) + self.gps.latitude_deg
                long2 = 0.0003 * math.cos(-self.imu.euler.z) + self.gps.longitude_deg
                self.compass_plot.update([self.gps.latitude_deg, lat2], [self.gps.longitude_deg, long2])
                if self.imu.system_status > 1:
                    self.compass_plot.set_properties(color='green')
                else:
                    self.compass_plot.set_properties(color='red')

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()


Animator().run()
