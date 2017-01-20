import math

from breezyslam.components import Laser
from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM, CoreSLAM

from PIL import Image

from atlasbuggy.robot.robotobject import RobotObject
from atlasbuggy.plotters.robotplot import RobotPlot


class LidarTurret(RobotObject):
    def __init__(self, enable_slam=True):
        self.current_tick = 0  # current tick received
        self.distances = []  # lidar distance readings in mm
        self.ticks = []  # tick readings in units of encoder counts
        self.current_index = 0  # index to add next piece of data to
        self.end_index = -1  # where the current set of data ends

        self.ticks_per_rotation = 0
        self.prev_timestamp = 0
        self.update_rate_hz = 0.0

        self.point_cloud_xs = []  # x points in mm
        self.point_cloud_ys = []  # y points in mm
        self.cloud_updated = False

        lidar_range = (-4000, 4000)
        self.point_cloud_plot = RobotPlot(
            "point cloud", x_range=lidar_range, y_range=lidar_range, range_offset=1
        )

        self.enable_slam = enable_slam
        self.laser = None
        self.slam = None
        self.slam_flag = SLAM.STATIONARY

        self.paused = False

        super(LidarTurret, self).__init__("lidar")

    def receive_first(self, packet):
        # <scan_size, scan_rate, detection_angle, distance_no_detection> (float)
        data = packet.split("\t")
        # data = list(map(float, data))
        # assert len(data) == 4

        scan_size = float(data[0])
        scan_rate_hz = float(data[1])
        detection_angle_degrees = float(data[2])
        distance_no_detection_mm = float(data[3])

        if self.enable_slam:
            self.laser = Laser(scan_size, scan_rate_hz, detection_angle_degrees, distance_no_detection_mm)
            self.slam = SLAM(self.laser, flag=self.slam_flag)
        print("initialized with data:", data)

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        if len(data) < 2:
            return False

        delta_tick = int(data[0])
        if data[1] == "> nack\r":
            return False
        distance = int(data[1]) * 10  # convert cm to mm

        self.current_tick += delta_tick

        if self.current_index >= len(self.distances):
            self.distances.append(distance)
            self.ticks.append(self.current_tick)
        else:
            self.distances[self.current_index] = distance
            self.ticks[self.current_index] = self.current_tick
        self.current_index += 1

        if len(data) == 3:
            self.end_index = self.current_index
            self.current_index = 0
            rotations = int(data[2])
            print(timestamp, rotations, self.current_tick, 1 / (timestamp - self.prev_timestamp), self.end_index)
            self.prev_timestamp = timestamp

            self.make_point_cloud()
            self.cloud_updated = True
            self.point_cloud_plot.update(self.point_cloud_xs[0:self.end_index],
                                         self.point_cloud_ys[0:self.end_index])

            self.current_tick = 0

            if rotations > 2 and self.enable_slam:
                self.update_slam(timestamp)

    def did_cloud_update(self):
        if self.cloud_updated:
            self.cloud_updated = False
            return True
        else:
            return False

    def make_point_cloud(self):
        self.ticks_per_rotation = self.current_tick
        if self.ticks_per_rotation > 0:
            for index in range(self.end_index):
                angle = self.ticks[index] / self.ticks_per_rotation * 2 * math.pi
                x = self.distances[index] * math.cos(angle)
                y = self.distances[index] * math.sin(angle)
                if index >= len(self.point_cloud_xs):
                    self.point_cloud_xs.append(x)
                else:
                    self.point_cloud_xs[index] = x

                if index >= len(self.point_cloud_ys):
                    self.point_cloud_ys.append(y)
                else:
                    self.point_cloud_ys[index] = y

    @property
    def point_cloud(self):
        return self.point_cloud_xs[:self.end_index], self.point_cloud_ys[:self.end_index]

    def update_slam(self, timestamp):
        if self.enable_slam:
            self.slam.update(timestamp, self.distances[0:self.end_index], self.ticks_per_rotation)

    def set_paused(self, paused=None):
        if paused is None:
            self.paused = not self.paused

        self.send("p%i" % int(self.paused))


class SLAM:
    """
    takes a breezyslam laser object and a flag to determine the
    slam algorithm that is used.
    """
    STATIONARY = 0
    MOVING = 1
    MOVING_ODOMETRY = 2

    def __init__(self, laser, map_pixels=800, map_size=32, flag=None):
        # def __init__(self, laser, map_pixels=0, map_size=0):
        self.laser = laser

        self.map_pixels = map_pixels
        self.map_size = map_size

        if flag is None:
            self.flag = SLAM.STATIONARY
        else:
            self.flag = flag

        # For odometry
        self.velocities = (0, 0, 0)

        self.algorithm = None
        self.set_algorithm()

    def set_algorithm(self):
        if self.flag == SLAM.STATIONARY:
            self.algorithm = Deterministic_SLAM(self.laser, self.map_pixels, self.map_size)
            print("using deterministic slam")
        elif self.flag == SLAM.MOVING:
            self.algorithm = RMHC_SLAM(self.laser, self.map_pixels, self.map_size)
            print("using RMHC slam")
        elif self.flag == SLAM.MOVING_ODOMETRY:
            self.algorithm = CoreSLAM(self.laser, self.map_pixels, self.map_size)
            print("using CoreSLAM slam")

    def update(self, timestamp, pointcloud, theta):
        # if self.flag != SLAM.MOVING_ODOMETRY:
        #     self.algorithm.update(pointcloud)
        # else:
        if self.flag == SLAM.MOVING_ODOMETRY:
            self.update_velocities(pointcloud, theta, timestamp)
        self.algorithm.update(pointcloud, self.velocities)

    def change_flag(self, flag):
        if flag != self.flag:
            self.flag = flag
            self.set_algorithm()

    def make_image(self, image_name):
        mapbytes = bytearray(self.map_size * self.map_pixels)
        self.algorithm.getmap(mapbytes)
        image = Image.frombuffer('L', (self.map_size, self.map_pixels), mapbytes, 'raw', 'L', 0, 1)
        image.save('%s.png' % image_name)

    def update_velocities(self, pointcloud, tetha, timestamp):
        """
        pretty much
        estimate dxy dtetha and dt comparing with the previous measurement
        needs implementation
        """
        pass
