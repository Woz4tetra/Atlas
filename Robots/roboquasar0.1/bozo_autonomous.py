# This file is meant to collect data from manual control. Terminal interface only
from atlasbuggy.interface.live import LiveRobot

from joysticks.wiiu_joystick import WiiUJoystick
from algorithms.controls.bozo_controller import BozoController
from roboquasar import RoboQuasar

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

roboquasar = RoboQuasar(True, "Autonomous test map 1", "Autonomous test map 1 inside border",
                        "Autonomous test map 1 outside border", "cut")


class DataCollector(LiveRobot):
    def __init__(self):
        self.manual_mode = True
        self.enable_plotting = True

        print("X: Switch between manual and autonomous\n"
              "R: Deadman switch. Hold this until something goes wrong\n"
              "L: Record checkpoint\n"
              "left stick: steering while in manual mode\n"
              "A: calibrate steering"
              )

        if self.enable_plotting:
            self.gps_plot = RobotPlot("gps")
            self.checkpoint_plot = RobotPlot("checkpoint", marker='.', linestyle='', markersize=8)
            self.map_plot = RobotPlot("map")
            self.inner_map_plot = RobotPlot("inner map")
            self.outer_map_plot = RobotPlot("outer map")
            self.compass_plot = RobotPlot("compass")
            self.goal_plot = RobotPlot("checkpoint goal")

            self.accuracy_check_plot = RobotPlotCollection("Animation", self.gps_plot, self.checkpoint_plot,
                                                           self.map_plot, self.inner_map_plot, self.outer_map_plot,
                                                           self.compass_plot, self.goal_plot)

            self.plotter = LivePlotter(1, self.accuracy_check_plot)

            self.map_plot.update(roboquasar.checkpoints.lats, roboquasar.checkpoints.longs)
            self.inner_map_plot.update(roboquasar.inner_map.lats, roboquasar.inner_map.longs)
            self.outer_map_plot.update(roboquasar.outer_map.lats, roboquasar.outer_map.longs)

            self.controller = BozoController(roboquasar.checkpoints, roboquasar.inner_map, roboquasar.outer_map, offset=5)

        super(DataCollector, self).__init__(
            *roboquasar.get_sensors(),
            joystick=WiiUJoystick(),
            log_data=True, log_dir=("data day 7", None), debug_prints=True
        )
        self.record("initial compass", roboquasar.compass_str)

    def start(self):
        roboquasar.brakes.unbrake()

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(roboquasar.gps):
            print("%2.5f" % timestamp)
            print(roboquasar.gps)
            print(roboquasar.imu)
            if roboquasar.gps.is_position_valid() and self.enable_plotting:
                self.gps_plot.append(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)

                if self.controller.is_point_inside(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg):
                    self.outer_map_plot.set_properties(color="blue")
                else:
                    self.outer_map_plot.set_properties(color="red")

                if self.controller.is_point_outside(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg):
                    self.inner_map_plot.set_properties(color="blue")
                else:
                    self.inner_map_plot.set_properties(color="red")

                if self.plotter.plot() is False:
                    return False

            self.update_steering()

        elif self.did_receive(roboquasar.imu):
            self.update_steering()

            if roboquasar.gps.is_position_valid() and self.enable_plotting:
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

        elif self.did_receive(roboquasar.steering):
            print("%2.5f" % timestamp)
            print(roboquasar.steering)
        elif self.did_receive(roboquasar.brakes):
            print("%2.5f" % timestamp)
            print(roboquasar.brakes)

    def update_steering(self):
        if roboquasar.gps.is_position_valid():
            if not self.controller.is_initialized():
                self.controller.initialize(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg)
                print("Initialized controller:", self.controller.map[self.controller.current_index])
            else:
                steering_angle = self.controller.update(roboquasar.gps.latitude_deg, roboquasar.gps.longitude_deg,
                                                        roboquasar.offset_angle())

                if not self.manual_mode:
                    goal_lat, goal_long = self.controller.map[self.controller.current_index]
                    roboquasar.steering.set_position(steering_angle)
                    print("goal offset:", goal_lat - roboquasar.gps.latitude_deg, goal_long - roboquasar.gps.longitude_deg)

                self.record("steering angle",
                            "%s\t%s\t%s" % (steering_angle, self.manual_mode, self.controller.current_index))

                if self.enable_plotting:
                    self.goal_plot.update(
                        [roboquasar.gps.latitude_deg, self.controller.map[self.controller.current_index][0]],
                        [roboquasar.gps.longitude_deg, self.controller.map[self.controller.current_index][1]])
        # else:
            # print("Skipping GPS value!!")

    def loop(self):
        if self.joystick is not None:
            if roboquasar.steering.calibrated and self.manual_mode:
                if self.joystick.axis_updated("left x"):
                    roboquasar.steering.set_speed(self.joystick.get_axis("left x"))
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    roboquasar.steering.calibrate()

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")
            elif self.joystick.button_updated("R") and self.joystick.get_button("R"):
                print("Current checkpoint:", roboquasar.checkpoint_num)
                print(roboquasar.gps)
                self.record("checkpoint", str(roboquasar.checkpoint_num))
                roboquasar.checkpoint_num += 1

            if self.joystick.button_updated("L"):
                if self.joystick.get_button("L"):
                    roboquasar.brakes.unbrake()
                else:
                    roboquasar.brakes.brake()
                    print("!!SWITCH RELEASED, BRAKING!!")

    def close(self, reason):
        if reason != "done":
            roboquasar.brakes.brake()
            print("!!EMERGENCY BRAKE!!")


DataCollector().run()
