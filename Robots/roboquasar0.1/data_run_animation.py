# Compare checkpoint data to checkpoint maps

from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from roboquasar import RoboQuasar, file_sets

roboquasar = RoboQuasar(False, "buggy course map")


class Animator(SimulatedRobot):
    def __init__(self):
        self.gps_plot = RobotPlot("gps")
        self.checkpoint_plot = RobotPlot("checkpoint", marker='.', linestyle='', markersize=8)
        self.map_plot = RobotPlot("map")
        self.compass_plot = RobotPlot("compass")

        self.map_plot.update(roboquasar.checkpoints.lats, roboquasar.checkpoints.longs)

        self.accuracy_check_plot = RobotPlotCollection("Animation", self.gps_plot, self.checkpoint_plot,
                                                       self.map_plot, self.compass_plot)

        self.plotter = LivePlotter(1, self.accuracy_check_plot)


        file_name, directory = file_sets["data day 6"][1]
        super(Animator, self).__init__(
            file_name, directory,
            *roboquasar.get_sensors()
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive("initial compass"):
            roboquasar.init_compass(packet)
            print("compass value:", packet)
        if roboquasar.gps.is_position_valid():
            if self.did_receive("checkpoint"):
                self.checkpoint_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(roboquasar.gps):
                self.gps_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(roboquasar.imu):
                angle = roboquasar.offset_angle()
                lat2, long2 = roboquasar.compass_coords(-angle)
                self.compass_plot.update([roboquasar.gps.latitude_deg, lat2],
                                         [roboquasar.gps.longitude_deg, long2])
                if roboquasar.imu.system_status > 1:
                    self.compass_plot.set_properties(color='green')
                else:
                    if roboquasar.compass_angle is None:
                        self.compass_plot.set_properties(color='red')
                    else:
                        self.compass_plot.set_properties(color='purple')

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()


Animator().run()
