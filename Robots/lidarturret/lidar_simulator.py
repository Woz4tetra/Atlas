from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.staticplotter import StaticPlotter

from lidar.lidarturret import LidarTurret


animate = False


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)

        if animate:
            self.animation = LivePlotter(1, self.turret.point_cloud_plot)
            self.animation.draw_dot("point cloud", 0, 0, color='orange', markersize=5)
        else:
            self.plotter = StaticPlotter(1, self.turret.cloud_time_plot)

        super(LidarPlotter, self).__init__(-1, -1, self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            # self.turret.time_plot1.append(timestamp, timestamp - self.turret.prev_time1)
            if animate and self.turret.did_cloud_update():
                if not self.animation.plot():
                    return False

    def close(self):
        if animate:
            self.animation.close()
        else:
            self.plotter.plot()
            self.plotter.show()


LidarPlotter().run()
