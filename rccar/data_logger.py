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

        pipeline = Pipeline(self.cam_width, self.cam_height, False)
        capture = Camera(self.cam_width, self.cam_height,
                         update_fn=lambda params: self.update_camera())

        super(DataLogger, self).__init__(pipeline, capture, log_data=True)

        # while not self.gps.get('fix'):
        #     print("waiting for fix...", self.gps)
        #     time.sleep(0.15)
        #
##        bearing = self.robot.filter.get_gps_bearing(
##            -71.420864, 42.427317, -71.420795, 42.427332
##        )
##        bearing = (-bearing - math.pi / 2) % (2 * math.pi)
        
        initial_long, initial_lat = self.checkpoints[-1]
        second_long, second_lat = self.checkpoints[0]

        bearing = self.robot.filter.get_gps_bearing(
            initial_long, initial_lat, second_long, second_lat
        )        
##        bearing = (-bearing - math.pi / 2) % (2 * math.pi)

        print(initial_long, initial_lat, bearing)
        self.robot.filter.initialize_filter(
            initial_long, initial_lat, bearing
        )
        self.robot.start()

    def main(self):
        state = self.robot.filter.state
        
        print("(%0.6f, %0.6f, %0.4f), (%0.6f, %0.6f)" % (
              state["x"], state["y"], state["angle"],
              self.robot.sensors['gps'].get('long'), self.robot.sensors['gps'].get('lat')), end="\r")
        time.sleep(0.05)


DataLogger().run()
