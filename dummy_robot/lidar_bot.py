import math
import time

import matplotlib.pyplot as plt
from robot.interface import RobotObject, RobotInterface


class LivePlotter:
    def __init__(self, xs, ys):
        plt.ion()

        self.fig = plt.figure()

        plt.axis([-500, 500, -500, 500])
        # plt.axis([0, len(xs), 0, 1000])
        ax = self.fig.add_subplot(111)

        self.lidar_cloud, = ax.plot(xs, ys, '.')
        ax.plot(0, 0, 'o', color='black', markersize=10)

    def plot_scatter(self, xs, ys):
        self.lidar_cloud.set_xdata(xs)
        self.lidar_cloud.set_ydata(ys)
        self.fig.canvas.draw()
        # plt.scatter(xs, ys)

        # plt.show()
        plt.pause(0.0001)


class LidarTurret(RobotObject):
    def __init__(self):
        self.ticks_per_rotation = None
        self.current_tick = 0
        self.rotations = 0
        self.distance = 0

        self.distances = [0] * self.ticks_per_rotation
        self.point_cloud = [(0, 0)] * self.ticks_per_rotation
        self.point_cloud_x = [0] * self.ticks_per_rotation
        self.point_cloud_y = [0] * self.ticks_per_rotation

        super(LidarTurret, self).__init__(
            "lidar",
            {
                "mac": "/dev/tty.usbmodem*",
                "linux": "/dev/tty.*"
            }
        )

    def get_point(self, distance):
        angle = 2 * math.pi * self.current_tick / self.ticks_per_rotation
        return distance * math.cos(angle), distance * math.sin(angle)

    def receive_first(self, packet):
        self.ticks_per_rotation = int(packet)

    def receive(self, packet):
        data = packet.split("\t")
        if len(data) == 2:
            self.current_tick = int(data[0])
            self.rotations = int(data[1])

        elif len(data) == 1:
            self.distance = int(data[0])
            x, y = self.get_point(self.distance)

            self.distances[self.current_tick] = self.distance
            self.point_cloud[self.current_tick] = (x, y)

            self.point_cloud_x[self.current_tick] = x
            self.point_cloud_y[self.current_tick] = y



class PyAccel(RobotObject):
    def __init__(self):
        self.x, self.y, self.z = 0, 0, 0
        super(PyAccel, self).__init__("pyaccel", "/dev/tty.usbmodem*")

    def receive(self, packet):
        data = packet.split("\t")
        if data[0] == "A":
            self.x = int(data[1])
            self.y = int(data[2])
            self.z = int(data[3])

    def set_led(self, *led_states):
        self.send(("%i\t" * 4) % led_states)


class LidarBot(RobotInterface):
    def __init__(self):
        self.lidar = LidarTurret()
        # self.pyaccel = PyAccel()

        self.counts = [x for x in range(self.lidar.ticks_per_rotation)]
        self.liveplot = LivePlotter(
            self.lidar.point_cloud_x,
            self.lidar.point_cloud_y
            # self.counts,
            # self.lidar.distances
        )

        super(LidarBot, self).__init__(
            self.lidar,
            # self.pyaccel,
            log_data=False)

    def loop(self):
        if self.lidar.did_update():
            self.liveplot.plot_scatter(
                self.lidar.point_cloud_x,
                self.lidar.point_cloud_y
                # self.counts,
                # self.lidar.distances
            )

        # if self.pyaccel.did_update():
        #     print(self.pyaccel.x, self.pyaccel.y, self.pyaccel.z)

        time.sleep(0.05)

    def close(self):
        plt.ioff()
        plt.close('all')
        plt.gcf()


LidarBot().run()
