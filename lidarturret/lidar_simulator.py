from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.staticplotter import StaticPlotter

from lidar.lidarturret import LidarTurret


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret()

        self.turret.time_plots.enabled = True
        self.plotter = StaticPlotter(1, self.turret.time_plots)

        super(LidarPlotter, self).__init__(-1, -1, self.turret)

    # def object_packet(self, timestamp):
    #     if self.did_receive(self.turret):
    #         self.turret.time_plot1.append(timestamp, timestamp - self.turret.prev_time1)

    def close(self):
        self.plotter.plot()
        self.plotter.show()


LidarPlotter().run()
