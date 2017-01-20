
from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.robotplot import RobotPlot
from atlasbuggy.plotters.staticplotter import StaticPlotter

from lidarturret import LidarTurret


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)

        # self.plotter = StaticPlotter(1, )

        super(LidarPlotter, self).__init__(-1, -1, self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            pass  # update plotter

    def close(self):
        pass
        # self.plotter.plot()
        # self.plotter.show()


LidarPlotter().run()
