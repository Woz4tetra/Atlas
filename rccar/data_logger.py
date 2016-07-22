import math
import time
from pprint import pprint

from buggypi.vision.camera import Camera
from vision.pipeline import Pipeline
from standard_runner import StandardRunner


class DataLogger(StandardRunner):
    def __init__(self):
        self.cam_width = 480
        self.cam_height = 320

        pipeline = Pipeline(self.cam_width, self.cam_height, True)
        capture = Camera(self.cam_width, self.cam_height,
                         lambda params: self.update_camera())

        super(DataLogger, self).__init__(pipeline, capture, log_data=True)

        # while not self.gps.get('fix'):
        #     print("waiting for fix...", self.gps)
        #     time.sleep(0.15)
        #
        # bearing = self.robot.filter.get_gps_bearing(
        #     -71.420864, 42.427317, -71.420795, 42.427332
        # )
        initial_long, initial_lat = self.checkpoints[-1]
        second_long, second_lat = self.checkpoints[0]

        bearing = self.robot.filter.get_gps_bearing(
            initial_long, initial_lat, second_long, second_lat
        )

        self.robot.filter.initialize_filter(
            self.gps.get('long'), self.gps.get('lat'), bearing
        )

    def main(self):
        state = self.robot.filter.state
        print("(%0.6f, %0.6f, %0.4f), (%0.6f, %0.6f)\r",
              state["x"], state["y"], state["angle"],
              self.gps['long'], self.gps['lat'])
        time.sleep(0.05)


DataLogger().run()
