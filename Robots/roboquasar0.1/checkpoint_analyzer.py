# Compare checkpoint data to checkpoint maps

from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from roboquasar import RoboQuasar, file_sets

from atlasbuggy.files.mapfile import MapMaker

roboquasar = RoboQuasar(False, "buggy course map")


class CheckpointAnalyzer(SimulatedRobot):
    def __init__(self):
        self.make_map = True

        self.gps_plot = RobotPlot("gps")
        self.checkpoint_plot = RobotPlot("checkpoint", marker='.', linestyle='', markersize=8)
        self.map_plot = RobotPlot("map")

        self.map_plot.update(roboquasar.checkpoints.lats, roboquasar.checkpoints.longs)

        self.accuracy_check_plot = RobotPlotCollection("Accuracy check", self.gps_plot, self.checkpoint_plot,
                                                       self.map_plot)

        self.plotter = StaticPlotter(1, self.accuracy_check_plot)

        file_name, directory = file_sets["data day 6"][1]
        super(CheckpointAnalyzer, self).__init__(
            file_name, directory,
            *roboquasar.get_sensors()
        )

        if self.make_map:
            self.comparison_maker = MapMaker(self.parser.file_name[:-5])
            # self.comparison_maker.extend(roboquasar.checkpoints)

    def received(self, timestamp, whoiam, packet, packet_type):
        if roboquasar.gps.is_position_valid():
            if self.did_receive("checkpoint"):
                self.checkpoint_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                if self.make_map:
                    self.comparison_maker.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
            elif self.did_receive(roboquasar.gps):
                self.gps_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)

    def close(self, reason):
        if self.make_map:
            self.comparison_maker.make_map()
            print("Wrote map:", self.comparison_maker.full_path)
        if reason != "error":
            self.plotter.plot()
            self.plotter.show()


CheckpointAnalyzer().run()
