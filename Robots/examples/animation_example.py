import math

from atlasbuggy.interface import simulate
from atlasbuggy.robot import Robot

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection

from robot_objects.adafruit_gps.adafruit_gps import AdafruitGPS
from robot_objects.bno055.bno055 import BNO055

file_sets = {
    "data day 8": (
        ("15;23", "2017_Feb_26"),  # 0, 3rd run, very good run. Did multiple laps. Joystick out of range test
        ("15;33", "2017_Feb_26"),  # 1, 4th run, multiple laps
    ),
    "data day 7": (
        ("17;33", "2017_Feb_24"),  # 0, Manual roll around course
        ("20;06", "2017_Feb_24"),  # 1, Walking home
        ("FIRST SUCCESSFUL RUN", "2017_Feb_24"),
    ),

}


class AnimationExample(Robot):
    def __init__(self):
        self.gps_plot = RobotPlot("gps", color="red")
        self.compass_plot = RobotPlot("compass", color="purple")
        self.sticky_compass_plot = RobotPlot("sticky compass", color="gray")

        self.sticky_compass_counter = 0
        self.sticky_compass_skip = 100

        self.accuracy_check_plot = RobotPlotCollection(
            "Animation", self.sticky_compass_plot, self.gps_plot, self.compass_plot
        )

        self.plotter = LivePlotter(2, self.accuracy_check_plot,
                                   matplotlib_events=dict(
                                       key_press_event=self.key_press
                                   ))

        self.gps = AdafruitGPS()
        self.imu = BNO055()

        # whoiam IDs were different in the old logs, changing for convenience, please don't do this otherwise!!
        self.gps.whoiam = "gps"
        self.imu.whoiam = "imu"

        # file_name, directory = file_sets["rolls day 3"][0]
        super(AnimationExample, self).__init__(
            self.imu, self.gps
        )

        self.compass_angle = 0.0
        self.start_angle = 0.0

        self.link(self.gps, self.receive_gps)
        self.link(self.imu, self.receive_imu)

    def key_press(self, event):
        if event.key == " ":
            self.toggle_pause()
            self.plotter.toggle_pause()

    def receive_gps(self, timestamp, packet, packet_type):
        if self.gps.is_position_valid():
            self.gps_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)

            status = self.plotter.plot()
            if status is not None:
                return status

    def init_compass(self, packet):
        self.compass_angle = math.radians(float(packet)) - math.pi / 2

    def offset_angle(self):
        if self.start_angle is None:
            self.start_angle = -self.imu.euler.z

        return (-self.imu.euler.z + self.start_angle - self.compass_angle) % (2 * math.pi)

    def compass_coords(self, angle, length=0.0003):
        lat2 = length * math.sin(angle) + self.gps.latitude_deg
        long2 = length * math.cos(angle) + self.gps.longitude_deg
        return lat2, long2

    def receive_imu(self, timestamp, packet, packet_type):
        if self.gps.is_position_valid():
            angle = self.offset_angle()
            lat2, long2 = self.compass_coords(angle)
            self.compass_plot.update([self.gps.latitude_deg, lat2],
                                     [self.gps.longitude_deg, long2])
            self.plotter.draw_text(
                self.accuracy_check_plot,
                "%0.4f" % angle,
                self.gps.latitude_deg, self.gps.longitude_deg,
                text_name="angle text"
            )

            if self.sticky_compass_skip > 0 and self.sticky_compass_counter % self.sticky_compass_skip == 0:
                lat2, long2 = self.compass_coords(angle, length=0.0001)
                self.sticky_compass_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
                self.sticky_compass_plot.append(lat2, long2)
                self.sticky_compass_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)
            self.sticky_compass_counter += 1

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive("initial compass"):
            self.init_compass(packet)
            print("compass value:", packet)

    def loop(self):
        if self.is_paused:
            status = self.plotter.plot()
            if status is not None:
                return status

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()
        print(self.dt())


animator_bot = AnimationExample()
file_name, directory = file_sets["data day 7"][-1]
simulate(file_name, directory, animator_bot)
