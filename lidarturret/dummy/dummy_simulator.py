from atlasbuggy import project
from atlasbuggy.plotter import PostPlotter

from dummy.dummy_bot import Dummy


class DummyPostPlotter(PostPlotter):
    def __init__(self, file_name, directory, enable_3d, use_pickled_data,
                 **plot_info):
        self.dummy = Dummy()

        super(DummyPostPlotter, self).__init__(
            file_name, directory, plot_info, enable_3d, use_pickled_data,
            self.dummy)

    def step(self, index, timestamp, whoiam, robot_object):
        if whoiam == self.dummy.whoiam:
            if not self.enable_3d:
                self.append_data(
                    "plot_dummy", robot_object.accel_x, robot_object.accel_y)
            else:
                self.append_data(
                    "plot_dummy", robot_object.accel_x, robot_object.accel_y,
                    robot_object.accel_z
                )


class TimestampPlotter(PostPlotter):
    def __init__(self, file_name, directory, enable_3d, use_pickled_data,
                 start_index=0, end_index=-1, **plot_info):
        self.dummy = Dummy()

        self.prev_log_t = 0
        self.prev_dummy_t = 0
        super(TimestampPlotter, self).__init__(
            file_name, directory, plot_info, enable_3d, use_pickled_data,
            start_index, end_index, self.dummy)

    def step(self, index, timestamp, whoiam, robot_object):
        if whoiam == self.dummy.whoiam:
            self.append_data(
                "plot_log_timestamps", index, timestamp - self.prev_log_t)
            self.append_data(
                "plot_dummy_timestamps", index, robot_object.dt - self.prev_dummy_t)

            self.prev_log_t = timestamp
            self.prev_dummy_t = robot_object.dt


def simulate_dummy():
    file_name, directory = project.parse_arguments(-1, -1)
    # DummyPostPlotter(file_name, directory,
    #                  enable_3d=True, use_pickled_data=False,
    #                  plot_dummy=dict(
    #                      color='red', label="dummy"
    #                  )).run()
    TimestampPlotter(file_name, directory,
                     enable_3d=False, use_pickled_data=False, start_index=0,  # end_index=500,
                     plot_log_timestamps=dict(
                         color='red', label="log"
                     ),
                     plot_dummy_timestamps=dict(
                         color='green', label="dummy"
                     )).run()
