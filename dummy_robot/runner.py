from atlasbuggy.interface import RobotInterface
from dummy.dummy_bot import Dummy
from joysticks.logitech import Logitech

live_plotting = False
if live_plotting:
    from atlasbuggy.plotters import LivePlotter


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()

        if live_plotting:
            data_range = (-90, 90)
            self.live_plot = LivePlotter(data_range, data_range, data_range, color='red', marker='+')

        super(DummyRunner, self).__init__(
            self.dummy,
            # joystick=Logitech(),
            log_data=False,
            debug_prints=True,
            port_updates_per_second=180
        )

    def packet_received(self, timestamp, whoiam):
        if whoiam == self.dummy.whoiam:
            if live_plotting:
                status = self.live_plot.plot(
                    self.dummy.accel_x,
                    self.dummy.accel_y,
                    self.dummy.accel_z
                )
                if not status:
                    return False

            else:
                # print(timestamp, self.dummy.dt, self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)
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