import math
import time
from pprint import pprint

from buggypi.vision.camera import Camera
from vision.pipeline import Pipeline
from standard_runner import StandardRunner


def get_gps_bearing(long, lat, prev_long, prev_lat):
    long = math.radians(long)
    lat = math.radians(lat)
    prev_long = math.radians(prev_long)
    prev_lat = math.radians(prev_lat)

    y = math.sin(long - prev_long) * math.cos(lat)
    x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
        prev_lat) * math.cos(lat) *
         math.cos(long - prev_long))

    bearing = math.atan2(y, x)
    bearing = (-bearing - math.pi / 2) % (2 * math.pi)
    return bearing


class DataLogger(StandardRunner):
    def __init__(self):
        self.cam_width = 480
        self.cam_height = 320

        pipeline = Pipeline(self.cam_width, self.cam_height, True)
        capture = Camera(self.cam_width, self.cam_height,
                         lambda params: self.update_camera())

        super(DataLogger, self).__init__(pipeline, capture, log_data=True)

        while not self.gps.get('fix'):
            print("waiting for fix...", self.gps)
            time.sleep(0.15)

        bearing = get_gps_bearing(
            -71.420864, 42.427317, -71.420795, 42.427332
        )
        self.robot.filter.initialize_filter(
            self.gps.get('long'), self.gps.get('lat'), bearing
        )

    def main(self):
        pprint(self.robot.get_state())


DataLogger().main()
