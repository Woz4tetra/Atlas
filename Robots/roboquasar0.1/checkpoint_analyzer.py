# Compare checkpoint data to checkpoint maps

from atlasbuggy.interface.simulated import SimulatedRobot

from sensors.gps import GPS
from sensors.imu import IMU

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from atlasbuggy.files.mapfile import MapFile, MapMaker

from roboquasar import *


class CheckpointAnalyzer(SimulatedRobot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.checkpoints = MapFile("02-17-2017 Test Run1")
        self.comparison_maker = MapMaker("comparison")
        # self.comparison_maker.extend(self.checkpoints)

        self.gps_plot = RobotPlot("gps")
        self.checkpoint_plot = RobotPlot("checkpoint")
        self.map_plot = RobotPlot("map")

        self.map_plot.update(self.checkpoints.lats, self.checkpoints.longs)

        self.accuracy_check_plot = RobotPlotCollection("Accuracy check", self.gps_plot, self.checkpoint_plot,
                                                       self.map_plot)

        self.plotter = StaticPlotter(1, self.accuracy_check_plot)

        file_name, directory = file_sets["data day 5"][-1]
        super(CheckpointAnalyzer, self).__init__(
            file_name, directory,
            self.gps, self.imu, self.steering, self.brakes, self.underglow
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.gps.is_position_valid():
            if self.did_receive("checkpoint"):
                self.checkpoint_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                self.comparison_maker.append(self.gps.latitude_deg, self.gps.longitude_deg)
            elif self.did_receive(self.gps):
                self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)

    def close(self):
        # self.comparison_maker.make_map()
        self.plotter.plot()
        self.plotter.show()


CheckpointAnalyzer().run()
