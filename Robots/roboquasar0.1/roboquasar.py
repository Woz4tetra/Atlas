import math

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow

from atlasbuggy.files.mapfile import MapFile


class RoboQuasar:
    def __init__(self, live, checkpoint_map_name=None, inner_map_name=None, outer_map_name=None, map_dir=None):

        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret(enabled=False)

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.checkpoint_num = 0
        if checkpoint_map_name is None:
            checkpoint_map_name = "buggy course map"
        if inner_map_name is None:
            inner_map_name = "buggy course map inside border"
        if outer_map_name is None:
            outer_map_name = "buggy course map outside border"
        if map_dir is None:
            map_dir = "buggy"

        self.checkpoints = MapFile(checkpoint_map_name, map_dir)
        self.inner_map = MapFile(inner_map_name, map_dir)
        self.outer_map = MapFile(outer_map_name, map_dir)

        self.compass_angle = 0
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

    def get_sensors(self):
        return self.gps, self.imu, self.turret, self.steering, self.brakes, self.underglow

    def init_compass(self, packet):
        self.compass_angle = math.radians(float(packet)) - math.pi / 2

    def offset_angle(self):
        # self.imu.euler.z *= 180 / math.pi
        if self.start_angle is None:
            self.start_angle = self.imu.euler.z

        return self.imu.euler.z - self.start_angle + self.compass_angle

    def compass_coords(self, angle):
        lat2 = 0.0003 * math.sin(angle) + self.gps.latitude_deg
        long2 = 0.0003 * math.cos(angle) + self.gps.longitude_deg
        return lat2, long2


file_sets = {
    "data day 7": (
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
    "data day 6"    : (
        ("16;47", "data_days/2017_Feb_18"),
        ("16;58", "data_days/2017_Feb_18"),
        ("18;15", "data_days/2017_Feb_18"),
    ),
    "data day 5"    : (
        ("16;49", "data_days/2017_Feb_17"),
        ("17;37", "data_days/2017_Feb_17"),
        ("18;32", "data_days/2017_Feb_17"),
    ),
    # "rolls day 4": (
    #
    # ),
    "data day 4"    : (
        ("15", "data_days/2017_Feb_14"),  # filter explodes, LIDAR interfered by the sun
        ("16;20", "data_days/2017_Feb_14"),  # shorten run, LIDAR collapsed
        ("16;57", "data_days/2017_Feb_14"),  # interfered LIDAR
        ("17;10", "data_days/2017_Feb_14"),  # all data is fine, interfered LIDAR
        ("17;33", "data_days/2017_Feb_14")),  # data is fine, normal run

    "data day 3"    : (
        ("16;38", "data_days/2017_Feb_08"),
        ("17", "data_days/2017_Feb_08"),
        ("18", "data_days/2017_Feb_08")),

    # no gyro values
    "trackfield"    : (
        ("15;46", "old_data/2016_Dec_02"),
        ("15;54", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02")),

    "rolls day 1"   : (
        ("07;22", "old_data/2016_Nov_06"),),  # bad gyro values

    "rolls day 2"   : (
        ("07;36;03 m", "old_data/2016_Nov_12"),
        ("09;12", "old_data/2016_Nov_12"),  # invalid values
        ("07;04;57", "old_data/2016_Nov_13")),  # bad gyro values

    "rolls day 3"   : (
        ("modified 07;04", "old_data/2016_Nov_13"),
        ("modified 07;23", "old_data/2016_Nov_13")),  # wonky value for mag.

    # rolling on the cut
    "first cut test": (
        ("16;29", "old_data/2016_Dec_09"),
        ("16;49", "old_data/2016_Dec_09"),
        ("16;5", "old_data/2016_Dec_09"),
        ("17;", "old_data/2016_Dec_09")),  # nothing wrong, really short

    "bad data"      : (
        ("16;07", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("16;09", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("18;00", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;02", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;09", "old_data/2016_Dec_09/bad_data")),  # gps spazzed out
}
