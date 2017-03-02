import time
import math

from atlasbuggy.interface.simulated import SimulatedRobot

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from algorithms.controls.bozo_controller import BozoController

from roboquasar import RoboQuasar, file_sets

using_old_data = True

if using_old_data:
    roboquasar = RoboQuasar(False)
else:
    roboquasar = RoboQuasar(
        False,
        "Autonomous test map 2",
        "Autonomous test map 2 inside border",
        "Autonomous test map 2 outside border",
        "cut"
    )


class Animator(SimulatedRobot):
    def __init__(self):
        self.gps_plot = RobotPlot("gps", color="red")
        self.checkpoint_plot = RobotPlot("checkpoint", color="green", marker='.', linestyle='', markersize=8)
        self.map_plot = RobotPlot("map", color="purple")
        self.inner_map_plot = RobotPlot("inner map")
        self.outer_map_plot = RobotPlot("outer map")
        self.compass_plot = RobotPlot("compass", color="purple")
        self.sticky_compass_plot = RobotPlot("sticky compass", color="gray")
        self.steering_plot = RobotPlot("steering angle", color="gray", plot_enabled=False)
        self.goal_plot = RobotPlot("checkpoint goal", color="cyan")
        self.recorded_goal_plot = RobotPlot("recorded checkpoint goal", "blue")

        self.imu_angle_plot = RobotPlot("imu angle")
        self.bearing_angle_plot = RobotPlot("bearing angle")
        self.filtered_angle_plot = RobotPlot("filtered angle")

        self.sticky_compass_counter = 0
        self.sticky_compass_skip = 100

        self.accuracy_check_plot = RobotPlotCollection("Animation", self.sticky_compass_plot, self.gps_plot,
                                                       self.checkpoint_plot,
                                                       self.map_plot, self.inner_map_plot, self.outer_map_plot,
                                                       self.steering_plot, self.compass_plot, self.goal_plot)
        self.angle_plots = RobotPlotCollection("Angle plots", self.imu_angle_plot, self.bearing_angle_plot,
                                               self.filtered_angle_plot, plot_enabled=False)

        self.plotter = LivePlotter(2, self.accuracy_check_plot, self.angle_plots, matplotlib_events={
            "key_press_event": self.key_press
        })

        self.map_plot.update(roboquasar.checkpoints.lats, roboquasar.checkpoints.longs)
        self.inner_map_plot.update(roboquasar.inner_map.lats, roboquasar.inner_map.longs)
        self.outer_map_plot.update(roboquasar.outer_map.lats, roboquasar.outer_map.longs)

        self.controller = BozoController(roboquasar.checkpoints, roboquasar.inner_map, roboquasar.outer_map, offset=5)

        file_name, directory = file_sets["rolls day 3"][0]
        # file_name, directory = file_sets["data day 8"][3]
        super(Animator, self).__init__(
            file_name, directory,
            *roboquasar.get_sensors()
        )

        self.link(roboquasar.gps, self.receive_gps)
        self.link(roboquasar.imu, self.receive_imu)

    def key_press(self, event):
        if event.key == " ":
            self.toggle_pause()
            self.plotter.toggle_pause()

    def receive_gps(self, timestamp, packet, packet_type):
        if roboquasar.gps.is_position_valid():
            roboquasar.update_bearing()
            if not self.controller.is_initialized():
                self.controller.initialize(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                print(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)

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

    def receive_imu(self, timestamp, packet, packet_type):
        if roboquasar.gps.is_position_valid():
            if using_old_data:
                roboquasar.imu.euler.z *= 180 / math.pi

            angle = roboquasar.offset_angle()
            lat2, long2 = roboquasar.compass_coords(angle)
            self.compass_plot.update([roboquasar.gps.latitude_deg, lat2],
                                     [roboquasar.gps.longitude_deg, long2])
            self.plotter.draw_text(
                self.accuracy_check_plot,
                "%0.4f" % angle,
                roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg,
                text_name="angle text"
            )

            self.imu_angle_plot.append(timestamp, roboquasar.imu_angle)
            self.bearing_angle_plot.append(timestamp, roboquasar.bearing)
            self.filtered_angle_plot.append(timestamp, angle)

            if self.sticky_compass_skip > 0 and self.sticky_compass_counter % self.sticky_compass_skip == 0:
                lat2, long2 = roboquasar.compass_coords(angle, length=0.0001)
                self.sticky_compass_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                self.sticky_compass_plot.append(lat2, long2)
                self.sticky_compass_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
            self.sticky_compass_counter += 1

            # if roboquasar.imu.system_status > 1:
            #     self.compass_plot.set_properties(color='green')
            # else:
            #     if roboquasar.compass_angle is None:
            #         self.compass_plot.set_properties(color='red')
            #     else:
            #         self.compass_plot.set_properties(color='purple')

            steering_angle = self.controller.update(
                roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg, roboquasar.offset_angle()
            )
            lat2, long2 = roboquasar.compass_coords(steering_angle + angle)
            self.steering_plot.update([roboquasar.gps.latitude_deg, lat2],
                                      [roboquasar.gps.longitude_deg, long2])

            self.goal_plot.update(
                [roboquasar.gps.latitude_deg, self.controller.map[self.controller.current_index][0]],
                [roboquasar.gps.longitude_deg, self.controller.map[self.controller.current_index][1]])

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive("initial compass"):
            roboquasar.init_compass(packet)
            print("compass value:", packet)
        elif self.did_receive("checkpoint"):
            self.checkpoint_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
        elif self.did_receive("steering angle"):
            goal_index = int(packet.split("\t")[-1])
            self.recorded_goal_plot.update([roboquasar.gps.latitude_deg, self.controller.map[goal_index][0]],
                                           [roboquasar.gps.longitude_deg, self.controller.map[goal_index][1]])

    def loop(self):
        if self.is_paused:
            if self.plotter.plot() is False:
                return False

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()
        print(self.dt())


Animator().run()
