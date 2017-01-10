from atlasbuggy.interface import RobotInterface
from dummy.dummy_bot import Dummy
from joysticks.logitech import Logitech

live_plotting = True
if live_plotting:
    from atlasbuggy.plotters import LivePlotter, RobotPlot


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()

        if live_plotting:
            gravity_range = (-90, 90)
            self.xyz_plot = RobotPlot("dummy xyz",
                                      flat_plot=False, skip_count=100, color="red",
                                      x_range=gravity_range, y_range=gravity_range, z_range=gravity_range)
            self.xtz_plot = RobotPlot("dummy xtz",
                                      flat_plot=False, skip_count=100, color="blue",
                                      x_range=gravity_range, z_range=gravity_range)
            self.container_plot = RobotPlot("container", plot_enabled=False,
                                            flat_plot=True, skip_count=200, color="green",
                                            linestyle="None", marker=".",
                                            x_range=gravity_range, y_range=gravity_range)

            self.xs = [0] * 180
            self.ys = [0] * 180

            self.live_plot = LivePlotter(2, self.xyz_plot, self.xtz_plot, self.container_plot)

        super(DummyRunner, self).__init__(
            self.dummy,
            # joystick=Logitech(),
            log_data=False,
            debug_prints=False,
            port_updates_per_second=180
        )

    def packet_received(self, timestamp, whoiam):
        if whoiam == self.dummy.whoiam:
            if live_plotting:
                self.xtz_plot.append(self.dummy.accel_x, self.dummy.dt, self.dummy.accel_z)
                self.xyz_plot.append(self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)

                if self.container_plot.enabled:
                    self.xs[self.dummy.accel_z % 180] = self.dummy.accel_x
                    self.ys[self.dummy.accel_z % 180] = self.dummy.accel_y
                self.container_plot.update(self.xs, self.ys)
                # print(timestamp, self.dummy.dt, self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)
                if not self.live_plot.plot():
                    return False
            else:
                print("Behind by %7.5fs (%7.5f, %7.5f)" % (timestamp - self.dummy.dt, timestamp, self.dummy.dt))

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
