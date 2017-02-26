from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from algorithms.controls.bozo_controller import BozoController

from roboquasar import RoboQuasar, file_sets

roboquasar = RoboQuasar(
    False,
    "Autonomous test map 2",
    "Autonomous test map 2 inside border",
    "Autonomous test map 2 outside border",
    "cut"
)


class Animator(SimulatedRobot):
    def __init__(self):
        self.gps_plot = RobotPlot("gps")
        self.checkpoint_plot = RobotPlot("checkpoint", marker='.', linestyle='', markersize=8)
        self.map_plot = RobotPlot("map")
        self.inner_map_plot = RobotPlot("inner map")
        self.outer_map_plot = RobotPlot("outer map")
        self.compass_plot = RobotPlot("compass")
        self.goal_plot = RobotPlot("checkpoint goal")
        self.recorded_goal_plot = RobotPlot("recorded checkpoint goal")

        self.accuracy_check_plot = RobotPlotCollection("Animation", self.gps_plot, self.checkpoint_plot,
                                                       self.map_plot, self.inner_map_plot, self.outer_map_plot,
                                                       self.compass_plot, self.goal_plot)

        self.plotter = LivePlotter(1, self.accuracy_check_plot)

        self.map_plot.update(roboquasar.checkpoints.lats, roboquasar.checkpoints.longs)
        self.inner_map_plot.update(roboquasar.inner_map.lats, roboquasar.inner_map.longs)
        self.outer_map_plot.update(roboquasar.outer_map.lats, roboquasar.outer_map.longs)

        self.controller = BozoController(roboquasar.checkpoints, roboquasar.inner_map, roboquasar.outer_map, offset=5)

        # file_name, directory = file_sets["rolls day 3"][0]
        file_name, directory = file_sets["data day 7"][-1]
        super(Animator, self).__init__(
            file_name, directory,
            *roboquasar.get_sensors()
        )

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive("initial compass"):
            roboquasar.init_compass(packet)
            print("compass value:", packet)
        if roboquasar.gps.is_position_valid():
            if not self.controller.is_initialized():
                self.controller.initialize(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                print(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)

            if self.did_receive("checkpoint"):
                self.checkpoint_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                if self.plotter.plot() is False:
                    return False
            elif self.did_receive(roboquasar.gps):
                self.gps_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)

                if self.plotter.plot() is False:
                    return False
                if self.controller.is_point_inside(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg):
                    self.outer_map_plot.set_properties(color="blue")
                else:
                    self.outer_map_plot.set_properties(color="red")

                if self.controller.is_point_outside(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg):
                    self.inner_map_plot.set_properties(color="blue")
                else:
                    self.inner_map_plot.set_properties(color="red")

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
                self.controller.update(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg,
                                       roboquasar.offset_angle())
                self.goal_plot.update(
                    [roboquasar.gps.latitude_deg, self.controller.map[self.controller.current_index][0]],
                    [roboquasar.gps.longitude_deg, self.controller.map[self.controller.current_index][1]])
            elif self.did_receive("steering angle"):
                goal_index = int(packet.split("\t")[-1])
                self.recorded_goal_plot.update([roboquasar.gps.latitude_deg, self.controller.map[goal_index][0]],
                                               [roboquasar.gps.longitude_deg, self.controller.map[goal_index][1]])

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()
        print(self.dt())


Animator().run()
