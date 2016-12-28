from atlasbuggy import project
from atlasbuggy.simulator import Simulator

from dummy_bot import Dummy


class DummySimulator(Simulator):
    def __init__(self, file_name, directory, enable_3d, use_pickled_data,
                 **plot_info):
        self.dummy = Dummy()
        super(DummySimulator, self).__init__(
            file_name, directory, plot_info, enable_3d, use_pickled_data,
            self.dummy)

    def step(self, index, timestamp, who_i_am, robot_object):
        if who_i_am == self.dummy.who_i_am:
            if not self.enable_3d:
                self.append_data(
                    "plot_dummy", robot_object.accel_x, robot_object.accel_y)
            else:
                self.append_data(
                    "plot_dummy", robot_object.accel_x, robot_object.accel_y,
                    robot_object.accel_z
                )


def simulate_dummy():
    file_name, directory = project.parse_arguments(-1, -1)
    DummySimulator(file_name, directory,
                   enable_3d=True, use_pickled_data=False,
                   plot_dummy=dict(
                       color='red', label="dummy"
                   )).run()


simulate_dummy()
