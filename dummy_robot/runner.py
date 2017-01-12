from atlasbuggy.robot.interface import RobotInterface
from dummy.dummy_bot import Dummy
from joysticks.logitech import Logitech

live_plotting = True
if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()

        if live_plotting:
            self.dummy.xyz_plot.max_length = 500
            self.dummy.xtz_plot.max_length = 500
            self.dummy.time_lag.max_length = 50
            self.dummy.queue_count.max_length = 50

            self.live_plot = LivePlotter(
                2, self.dummy.xyz_plot, self.dummy.xtz_plot, self.dummy.container_plot,
                self.dummy.time_plot,
                legend_args=dict(loc="upper left")
            )

        super(DummyRunner, self).__init__(
            self.dummy,
            # joystick=Logitech(),
            # log_data=False,
            # debug_prints=True,
        )

    def start(self):
        self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if whoiam == self.dummy.whoiam:
            if live_plotting:
                if self.live_plot.should_update(timestamp) and self.queue_len() < 10:
                    self.dummy.xtz_plot.append(self.dummy.accel_x, self.dummy.dt, self.dummy.accel_z)
                    self.dummy.xyz_plot.append(self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)

                    if self.dummy.container_plot.enabled:
                        self.dummy.xs[self.dummy.accel_z % 180] = self.dummy.accel_x
                        self.dummy.ys[self.dummy.accel_z % 180] = self.dummy.accel_y
                    self.dummy.container_plot.update(self.dummy.xs, self.dummy.ys)

                    self.dummy.time_lag.append(self.dt, timestamp - self.dummy.dt)
                    self.dummy.queue_count.append(self.dt, self.queue_len())

                    if not self.live_plot.plot():
                        return False
                self.record("time lag", "%7.5f\t%7.5f" % (timestamp, self.dummy.dt))
            else:
                print(self.queue_len())

                # else:
            #     print("Behind by %7.5fs (%7.5f, %7.5f)" % (timestamp - self.dummy.dt, timestamp, self.dummy.dt))

    def loop(self):
        if self.joystick is not None:
            if self.joystick.button_updated('A'):
                self.dummy.set_led('r', self.joystick.get_button('A'))
            if self.joystick.button_updated('B'):
                self.dummy.set_led('b', 255 * int(self.joystick.get_button('B')))
            if self.joystick.button_updated('X'):
                self.dummy.set_led('g', self.joystick.get_button('X'))
            if self.joystick.button_updated('Y'):
                self.dummy.set_led('y', self.joystick.get_button('Y'))

            if self.joystick.axis_updated("right y"):
                self.dummy.set_led('b', abs(
                    int(self.joystick.get_axis("right y") * 255)))

    def close(self):
        if live_plotting:
            self.live_plot.close()


def run_dummy():
    DummyRunner().run()


run_dummy()
