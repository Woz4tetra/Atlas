from breezyslam.components import Laser
from breezyslam.algorithms import RMHC_SLAM

from PIL import Image


class SLAM:
    """
    takes a breezyslam laser object and a flag to determine the
    slam algorithms that is used.
    """

    def __init__(self, point_cloud_size, detection_angle_degrees):
        self.map_size_pixels = 1600
        self.map_size_meters = 10
        self.trajectory = []
        self.mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)

        self.laser = Laser(point_cloud_size, 2.4, detection_angle_degrees, 0)
        self.algorithm = RMHC_SLAM(self.laser, self.map_size_pixels, self.map_size_meters)

    def update(self, timestamp, point_cloud):
        self.algorithm.update(point_cloud, (0, 0, 1 / self.laser.scan_rate_hz))
        print(self.algorithm.getpos())
        x_mm, y_mm, theta_degrees = self.algorithm.getpos()
        self.trajectory.append((x_mm, y_mm))
        self.algorithm.getmap(self.mapbytes)

    def make_image(self):
        self.algorithm.getmap(self.mapbytes)
        for coords in self.trajectory:
            x_mm, y_mm = coords

            x_pix = self.mm2pix(x_mm)
            y_pix = self.mm2pix(y_mm)

            self.mapbytes[y_pix * self.map_size_pixels + x_pix] = 0

        image = Image.frombuffer('L', (self.map_size_pixels, self.map_size_pixels), self.mapbytes, 'raw', 'L', 0, 1)
        image.save("map.png")

    def get_pos(self):
        return self.algorithm.getpos()

    def mm2pix(self, mm):
        return int(mm / (self.map_size_meters * 1000 / self.map_size_pixels))
