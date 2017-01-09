from atlasbuggy import project
from atlasbuggy.plotters import StaticPlotter
from atlasbuggy.interface import RobotInterfaceSimulator

from dummy.dummy_bot import Dummy


class DummySimulator(RobotInterfaceSimulator):
    def __init__(self, file_name, directory, enable_3d, use_pickled_data, start_index=0, end_index=-1,
                 **plot_info):
        self.dummy = Dummy()

        super(DummySimulator, self).__init__(
            file_name, directory, start_index, end_index,
            self.dummy
        )

        self.plotter = StaticPlotter(self.parser, plot_info, enable_3d, use_pickled_data)

    def packet_received(self, timestamp, whoiam):
        if whoiam == self.dummy.whoiam:
            if not self.plotter.enable_3d:
                self.plotter.append_data(
                    "plot_dummy", self.dummy.accel_x, self.dummy.accel_y)
            else:
                self.plotter.append_data(
                    "plot_dummy", self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z
                )
        return True

    def close(self):
        self.plotter.plot()


def simulate_dummy():
    file_name, directory = project.parse_arguments(-1, -1)
    DummySimulator(file_name, directory,
                   enable_3d=True, use_pickled_data=False,
                   plot_dummy=dict(
                         color='red', label="dummy"
                     )).run()
