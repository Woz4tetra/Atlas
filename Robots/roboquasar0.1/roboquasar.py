import math

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from algorithms.controls.bozo_controller import BozoController

from atlasbuggy.files.mapfile import MapFile
from atlasbuggy.robot import Robot
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.staticplotter import StaticPlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection


class RoboQuasar(Robot):
    def __init__(self, live, enable_plotting, animate,
                 checkpoint_map_name, inner_map_name, outer_map_name, map_dir):

        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret(enabled=False)

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        super(RoboQuasar, self).__init__(self.gps, self.imu, self.turret, self.steering, self.brakes, self.underglow)

        self.checkpoints = MapFile(checkpoint_map_name, map_dir)
        self.inner_map = MapFile(inner_map_name, map_dir)
        self.outer_map = MapFile(outer_map_name, map_dir)

        self.compass_angle = None
        self.start_angle = None
        self.compass_str = ""
        if live:
            while len(self.compass_str) == 0:
                self.compass_str = input("iPhone compass reading to record (degrees): ")
                try:
                    float(self.compass_str)
                except ValueError:
                    print("Input not a number...")
            self.init_compass(self.compass_str)

        self.manual_mode = True

        self.lat_data = []
        self.long_data = []
        self.bearing = None
        self.imu_angle = None

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

        if animate:
            self.plotter = LivePlotter(
                2, self.accuracy_check_plot, self.angle_plots,
                matplotlib_events=dict(key_press_event=self.key_press),
                enabled=enable_plotting
            )
        else:
            self.plotter = StaticPlotter(
                2, self.accuracy_check_plot, self.angle_plots,
                matplotlib_events=dict(key_press_event=self.key_press),
                enabled=enable_plotting
            )

        self.map_plot.update(self.checkpoints.lats, self.checkpoints.longs)
        self.inner_map_plot.update(self.inner_map.lats, self.inner_map.longs)
        self.outer_map_plot.update(self.outer_map.lats, self.outer_map.longs)

        self.controller = BozoController(self.checkpoints, self.inner_map, self.outer_map, offset=5)

        self.link(self.gps, self.receive_gps)
        self.link(self.imu, self.receive_imu)

    def receive_gps(self, timestamp, packet, packet_type):
        if not self.plotter.enabled:
            print("%0.2f" % timestamp, self.gps)
            print("%0.2f" % timestamp, self.imu)
        if self.gps.is_position_valid():
            # self.update_bearing()

            if not self.controller.is_initialized():
                self.controller.initialize(self.gps.latitude_deg, self.gps.longitude_deg)
                print(self.gps.latitude_deg, self.gps.longitude_deg)

            if self.plotter.enabled:
                self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)

                if isinstance(self.plotter, LivePlotter):
                    status = self.plotter.plot()
                    if status is not None:
                        return status

                    if self.controller.is_point_inside(self.gps.latitude_deg, self.gps.longitude_deg):
                        self.outer_map_plot.set_properties(color="blue")
                    else:
                        self.outer_map_plot.set_properties(color="red")

                    if self.controller.is_point_outside(self.gps.latitude_deg, self.gps.longitude_deg):
                        self.inner_map_plot.set_properties(color="blue")
                    else:
                        self.inner_map_plot.set_properties(color="red")

    def receive_imu(self, timestamp, packet, packet_type):
        if self.gps.is_position_valid() and self.compass_angle is not None:
            angle = self.offset_angle()

            steering_angle = self.controller.update(
                self.gps.latitude_deg, self.gps.longitude_deg, angle
            )
            self.record("steering angle",
                        "%s\t%s\t%s" % (steering_angle, self.manual_mode, self.controller.current_index))

            if not self.manual_mode:
                self.steering.set_position(steering_angle)

            if self.plotter.enabled:
                if isinstance(self.plotter, LivePlotter):
                    lat2, long2 = self.compass_coords(angle)
                    self.compass_plot.update([self.gps.latitude_deg, lat2],
                                             [self.gps.longitude_deg, long2])
                    self.plotter.draw_text(
                        self.accuracy_check_plot,
                        # "%0.4f, %s" % (math.degrees(steering_angle), self.steering.sent_step),
                        "%s, %s" % (self.steering.goal_step, self.steering.sent_step),
                        # "%0.4f" % angle,
                        self.gps.latitude_deg, self.gps.longitude_deg,
                        text_name="angle text"
                    )

                self.imu_angle_plot.append(timestamp, self.imu_angle)
                # self.bearing_angle_plot.append(timestamp, self.bearing)
                self.filtered_angle_plot.append(timestamp, angle)

                if self.sticky_compass_skip > 0 and self.sticky_compass_counter % self.sticky_compass_skip == 0:
                    lat2, long2 = self.compass_coords(angle, length=0.0001)
                    self.sticky_compass_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                    self.sticky_compass_plot.append(lat2, long2)
                    self.sticky_compass_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                self.sticky_compass_counter += 1

                if isinstance(self.plotter, LivePlotter):
                    lat2, long2 = self.compass_coords(steering_angle + angle)
                    self.steering_plot.update([self.gps.latitude_deg, lat2],
                                              [self.gps.longitude_deg, long2])

                    self.goal_plot.update(
                        [self.gps.latitude_deg, self.controller.map[self.controller.current_index][0]],
                        [self.gps.longitude_deg, self.controller.map[self.controller.current_index][1]])

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive("initial compass"):
            self.init_compass(packet)
            print("compass value:", packet)
        elif self.did_receive("checkpoint"):
            if self.gps.is_position_valid():
                self.checkpoint_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
        elif self.did_receive("steering angle"):
            if isinstance(self.plotter, LivePlotter):
                goal_index = int(packet.split("\t")[-1])
                self.recorded_goal_plot.update([self.gps.latitude_deg, self.controller.map[goal_index][0]],
                                               [self.gps.longitude_deg, self.controller.map[goal_index][1]])

    def loop(self):
        if self.is_paused and isinstance(self.plotter, LivePlotter):
            status = self.plotter.plot()
            if status is not None:
                return status

        if self.joystick is not None:
            if self.steering.calibrated and self.manual_mode:
                if self.joystick.axis_updated("left x") or self.joystick.axis_updated("left y"):
                    mag = math.sqrt(self.joystick.get_axis("left y") ** 2 + self.joystick.get_axis("left x") ** 2)
                    if mag > 0.5:
                        angle = math.atan2(self.joystick.get_axis("left y"), self.joystick.get_axis("left x"))
                        angle -= math.pi / 2
                        if angle < -math.pi:
                            angle += 2 * math.pi
                        self.steering.set_position(angle / 4)
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    self.steering.calibrate()
                elif self.joystick.dpad_updated():
                    if self.joystick.dpad[0] != 0:
                        self.steering.change_position(-self.joystick.dpad[0] * 10)

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")

            elif self.joystick.button_updated("L"):
                if self.joystick.get_button("L"):
                    self.brakes.unbrake()
                else:
                    self.brakes.brake()
                    print("\n\n!!SWITCH RELEASED, BRAKING!!\n\n")
            elif self.joystick.button_updated("R") and self.joystick.get_button("R"):
                self.brakes.toggle()

    def init_compass(self, packet):
        self.compass_angle = math.radians(float(packet)) - math.pi / 2

    def offset_angle(self):
        if self.start_angle is None:
            self.start_angle = -self.imu.euler.z

        self.imu_angle = (-self.imu.euler.z + self.start_angle - self.compass_angle) % (2 * math.pi)

        if self.bearing is None:
            return self.imu_angle
        else:
            if self.bearing - self.imu_angle > math.pi:
                self.imu_angle += 2 * math.pi
            if self.imu_angle - self.bearing > math.pi:
                self.bearing += 2 * math.pi

            return 0.5 * self.imu_angle + 0.5 * self.bearing
            # return self.imu_angle
            # return self.bearing
        # return self.imu_angle

    def update_bearing(self):
        if len(self.long_data) == 0 or self.gps.longitude_deg != self.long_data[-1]:
            self.long_data.append(self.gps.longitude_deg)
        if len(self.lat_data) == 0 or self.gps.latitude_deg != self.lat_data[-1]:
            self.lat_data.append(self.gps.latitude_deg)

        self.bearing = -math.atan2(self.gps.longitude_deg - self.long_data[0],
                                   self.gps.latitude_deg - self.lat_data[0]) + math.pi
        self.bearing = (self.bearing - math.pi / 2) % (2 * math.pi)

        if len(self.long_data) > 6:
            self.long_data.pop(0)
        if len(self.lat_data) > 6:
            self.lat_data.pop(0)

    def compass_coords(self, angle, length=0.0003):
        lat2 = length * math.sin(angle) + self.gps.latitude_deg
        long2 = length * math.cos(angle) + self.gps.longitude_deg
        return lat2, long2

    def key_press(self, event):
        if event.key == " ":
            self.toggle_pause()
            if isinstance(self.plotter, LivePlotter):
                self.plotter.toggle_pause()

    def close(self, reason):
        if reason == "done":
            if isinstance(self.plotter, LivePlotter):
                self.plotter.freeze_plot()
        else:
            self.brakes.brake()
            print("!!EMERGENCY BRAKE!!")

        self.plotter.close()
        print("Ran for %0.4fs" % self.dt())


map_sets = {
    "cut"  : (
        "Autonomous test map 2",
        "Autonomous test map 2 inside border",
        "Autonomous test map 2 outside border",
        "cut"
    ),
    "buggy": (
        "buggy course map",
        "buggy course map inside border",
        "buggy course map outside border",
        "buggy",
    )
}

file_sets = {
    "data day 9"        : (
        ("15;13", "data_days/2017_Mar_05"),  # 0
        ("15;19", "data_days/2017_Mar_05"),  # 1
        ("15;25", "data_days/2017_Mar_05"),  # 2
        ("15;27", "data_days/2017_Mar_05"),  # 3, running into the lamppost
        ("15;37", "data_days/2017_Mar_05"),  # 4, !! Weird errors !! not sure what's going on
        ("15;46", "data_days/2017_Mar_05"),  # 5
        ("16;00", "data_days/2017_Mar_05"),  # 6, Sketchy run
        ("16;05", "data_days/2017_Mar_05"),  # 7
        ("16;06", "data_days/2017_Mar_05"),  # 8, Weird error again, pointing 180º the wrong way
        ("16;11", "data_days/2017_Mar_05"),  # 9, Steering severely limited
        ("16;29", "data_days/2017_Mar_05"),  # 10, Walking home. UC offered interesting interference
    ),
    "moving to high bay": (
        ("14;08;26", "data_days/2017_Mar_02"),
    ),
    "data day 8"        : (
        ("15;05", "data_days/2017_Feb_26"),  # 0, 1st autonomous run
        ("15;11", "data_days/2017_Feb_26"),  # 1, 2nd autonomous run, IMU stopped working
        ("15;19", "data_days/2017_Feb_26"),  # 2, finished 2nd run
        ("15;23", "data_days/2017_Feb_26"),  # 3, 3rd run, very good run. Did multiple laps. Joystick out of range test
        ("15;33", "data_days/2017_Feb_26"),  # 4, 4th run, multiple laps
        ("15;43", "data_days/2017_Feb_26"),  # 5, walking home
    ),
    "data day 7"        : (
        ("14;19", "data_days/2017_Feb_24"),  # 0, rolling on schlenley 1
        ("14;26", "data_days/2017_Feb_24"),  # 1, rolling on schlenley 2
        ("16;13", "data_days/2017_Feb_24"),  # 2, GPS not found error 1
        ("16;14", "data_days/2017_Feb_24"),  # 3, Moving to the cut part 1
        ("16;21", "data_days/2017_Feb_24"),  # 4, Moving to the cut part 2
        ("16;49", "data_days/2017_Feb_24"),  # 5, GPS not found error 2
        ("17;09", "data_days/2017_Feb_24"),  # 6, Faulty checkpoint lock ons
        ("17;33", "data_days/2017_Feb_24"),  # 7, Manual roll around course
        ("20;06", "data_days/2017_Feb_24"),  # 8, Walking home
        ("FIRST SUCCESSFUL RUN", "data_days/2017_Feb_24"),
    ),

    # started using checkpoints
    "data day 6"        : (
        ("16;47", "data_days/2017_Feb_18"),
        ("16;58", "data_days/2017_Feb_18"),
        ("18;15", "data_days/2017_Feb_18"),
    ),
    "data day 5"        : (
        ("16;49", "data_days/2017_Feb_17"),
        ("17;37", "data_days/2017_Feb_17"),
        ("18;32", "data_days/2017_Feb_17"),
    ),
    # "rolls day 4": (
    #
    # ),
    "data day 4"        : (
        ("15", "data_days/2017_Feb_14"),  # filter explodes, LIDAR interfered by the sun
        ("16;20", "data_days/2017_Feb_14"),  # shorten run, LIDAR collapsed
        ("16;57", "data_days/2017_Feb_14"),  # interfered LIDAR
        ("17;10", "data_days/2017_Feb_14"),  # all data is fine, interfered LIDAR
        ("17;33", "data_days/2017_Feb_14")),  # data is fine, normal run

    "data day 3"        : (
        ("16;38", "data_days/2017_Feb_08"),
        ("17", "data_days/2017_Feb_08"),
        ("18", "data_days/2017_Feb_08")),

    # no gyro values
    "trackfield"        : (
        ("15;46", "old_data/2016_Dec_02"),
        ("15;54", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02")),

    "rolls day 1"       : (
        ("07;22", "old_data/2016_Nov_06"),),  # bad gyro values

    "rolls day 2"       : (
        ("07;36;03 m", "old_data/2016_Nov_12"),
        ("09;12", "old_data/2016_Nov_12"),  # invalid values
        ("07;04;57", "old_data/2016_Nov_13")),  # bad gyro values

    "rolls day 3"       : (
        ("modified 07;04", "old_data/2016_Nov_13"),
        ("modified 07;23", "old_data/2016_Nov_13")),  # wonky value for mag.

    # rolling on the cut
    "first cut test"    : (
        ("16;29", "old_data/2016_Dec_09"),
        ("16;49", "old_data/2016_Dec_09"),
        ("16;5", "old_data/2016_Dec_09"),
        ("17;", "old_data/2016_Dec_09")),  # nothing wrong, really short

    "bad data"          : (
        ("16;07", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("16;09", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("18;00", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;02", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;09", "old_data/2016_Dec_09/bad_data")),  # gps spazzed out
}
