import math
import time

# from autobuggy.vision.rccamera import RcCamera
# from pipeline import Pipeline
from standard_robot import StandardRobot


class DataLogger(StandardRobot):
    def __init__(self):
        self.cam_width = 480
        self.cam_height = 320
        # pipeline = Pipeline(self.cam_width, self.cam_height, False)
        # capture = RcCamera(self.cam_width, self.cam_height,
        #                    update_fn=lambda params: self.update_camera())
        pipeline = None
        capture = None

        super(DataLogger, self).__init__(pipeline, capture, log_data=True)

        # while not self.gps.get('fix'):
        #     print("waiting for fix...", self.gps)
        #     time.sleep(0.15)

        # initial_long, initial_lat = self.checkpoints[-1]
        # second_long, second_lat = self.checkpoints[0]
        #
        # bearing = self.filter.get_gps_bearing(
        #     initial_long, initial_lat, second_long, second_lat
        # )
        # bearing = (-bearing - math.pi / 2) % (2 * math.pi)
        #
        # print(initial_long, initial_lat, bearing)
        # self.filter.initialize_filter(
        #     initial_long, initial_lat, bearing
        # )
        self.start()

    def main(self):
        # state = self.filter.update_all(
        #     time.time() - self.time_start, self.encoder.get("counts"),
        #     self.yaw.get("yaw"), self.gps.get("long"), self.gps.get("lat"))
        # state = self.filter.state

        time.sleep(0.05)


DataLogger().run()
