from breezyslam.components import Laser
from breezyslam.algorithms import RMHC_SLAM, Deterministic_SLAM

from PIL import Image


class SLAM:
    """
    takes a breezyslam laser object and a flag to determine the
    slam algorithms that is used.
    """

    def __init__(self, lidar_turret):
        self.map_size_pixels = 1600
        self.map_size_meters = 50
        self.trajectory = []
        self.mapbytes = bytearray(self.map_size_pixels * self.map_size_pixels)

        self.laser = Laser(lidar_turret.point_cloud_size, 2.4, lidar_turret.detection_angle_degrees, 0)
        self.algorithm = RMHC_SLAM(self.laser, self.map_size_pixels, self.map_size_meters)

    def update(self, timestamp, point_cloud, velocity):
        self.algorithm.update(point_cloud, velocity)

        x_mm, y_mm, theta_degrees = self.algorithm.getpos()
        self.trajectory.append((x_mm, y_mm))

        self.algorithm.getmap(self.mapbytes)

    def make_image(self, image_name, image_format="pgm"):
        self.algorithm.getmap(self.mapbytes)
        for coords in self.trajectory:
            x_mm, y_mm = coords

            x_pix = self.mm2pix(x_mm)
            y_pix = self.mm2pix(y_mm)

            self.mapbytes[y_pix * self.map_size_pixels + x_pix] = 0

        if image_format == "pgm":
            pgm_save(image_name + "." + image_format, self.mapbytes,
                     (self.map_size_pixels, self.map_size_pixels))
        else:
            image = Image.frombuffer('L', (self.map_size_pixels, self.map_size_pixels), self.mapbytes, 'raw', 'L', 0, 1)
            image.save(image_name + "." + image_format)

    def get_pos(self):
        return self.algorithm.getpos()

    def mm2pix(self, mm):
        return int(mm / (self.map_size_meters * 1000 / self.map_size_pixels))


def pgm_load(filename):
    print('Loading image from file %s...' % filename)

    fd = open(filename, 'rt')

    # Skip constant header
    fd.readline()

    # Grab image size (assume square)
    imgsize = [int(tok) for tok in fd.readline().split()]

    # Start with empty list
    imglist = []

    # Read lines and append them to list until done
    while True:

        line = fd.readline()

        if len(line) == 0:
            break

        imglist.extend([int(tok) for tok in line.split()])

    fd.close()

    # Convert list into bytes
    imgbytes = bytearray(imglist)

    return imgbytes, imgsize


def pgm_save(filename, imgbytes, imgsize):
    print('\nSaving image to file %s' % filename)

    output = open(filename, 'wt')

    output.write('P2\n%d %d 255\n' % imgsize)

    wid, hgt = imgsize

    for y in range(hgt):
        for x in range(wid):
            output.write('%d ' % imgbytes[y * wid + x])
        output.write('\n')

    output.close()
