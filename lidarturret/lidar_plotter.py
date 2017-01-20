import numpy as np

from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from atlasbuggy.plotters.robotplot import RobotPlot
from atlasbuggy.plotters.staticplotter import StaticPlotter

from lidarturret import LidarTurret

class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret(662)

        self.light_plot = RobotPlot(
            "light",
            # plot_enabled=False,
        )
        self.delta_plot = RobotPlot(
            "delta",
            # plot_enabled=False,
        )

        self.plotter = StaticPlotter(1, self.light_plot, self.delta_plot)

        super(LidarPlotter, self).__init__(-1, -1, self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            self.light_plot.append(timestamp, self.turret.light_val)

    def close(self):
        light_vals = np.array(self.light_plot.data[1])
        deltas = np.diff(light_vals)
        self.delta_plot.update(self.light_plot.data[0][:-1], deltas)

        self.plotter.plot()
        self.plotter.show()

LidarPlotter().run()