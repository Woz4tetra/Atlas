from atlasbuggy.interface import RobotInterface
from dummy.dummy_bot import Dummy
from joysticks.logitech import Logitech

live_plotting = True
if live_plotting:
    from atlasbuggy.plotter import LivePlotter


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()

        if live_plotting:
            data_range = (-90, 90)
            self.live_plot = LivePlotter(data_range, data_range, data_range)

        super(DummyRunner, self).__init__(self.dummy, joystick=Logitech(),
                                          log_data=False, debug_prints=False)

    def loop(self):
        if self.dummy.did_update():
            if live_plotting:
                status = self.live_plot.plot(
                    self.dummy.accel_x,
                    self.dummy.accel_y,
                    self.dummy.accel_z
                )
                if not status:
                    return False
        if self.joystick.button_updated('A'):
            self.dummy.set_led('r', self.joystick.get_button('A'))

        if self.joystick.axis_updated("right y"):
            self.dummy.set_led('b', abs(
                int(self.joystick.get_axis("right y") * 255)))

    def close(self):
        if live_plotting:
            self.live_plot.close()


def run_dummy():
    DummyRunner().run()


