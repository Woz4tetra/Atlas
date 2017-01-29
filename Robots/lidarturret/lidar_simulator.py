from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.liveplotter import LivePlotter

from lidar.lidarturret import LidarTurret


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)

        self.animation = LivePlotter(1, self.turret.point_cloud_plot)
        self.animation.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarPlotter, self).__init__(-1, -1, self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            # self.turret.time_plot1.append(timestamp, timestamp - self.turret.prev_time1)
            if self.turret.did_cloud_update():
                if not self.animation.plot():
                    return False

    def close(self):
        self.animation.close()


LidarPlotter().run()
