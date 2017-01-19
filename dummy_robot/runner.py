from atlasbuggy.robot.interface import RobotInterface
from dummy.dummy1 import Dummy1
from dummy.dummy2 import Dummy2
from dummy.dummy3 import Dummy3
from joysticks.logitech import Logitech

live_plotting = True
if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy1 = Dummy1()
        self.dummy2 = Dummy2()
        self.dummy3 = Dummy3()

        if live_plotting:
            self.dummy1.xyz_plot.max_length = 25
            self.dummy1.xtz_plot.max_length = 25

            self.dummy1.time_lag.max_length = 50
            self.dummy1.queue_count.max_length = 50

            self.live_plot = LivePlotter(
                2, self.dummy1.xyz_plot, self.dummy1.xtz_plot, self.dummy1.container_plot,
                self.dummy1.time_plot,
                legend_args=dict(loc="upper left")
            )

        super(DummyRunner, self).__init__(
            self.dummy1,
            self.dummy2,
            self.dummy3,
            joystick=Logitech(),
            log_data=False,
            # debug_prints=True,
        )

    def start(self):   
        self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.dummy1):
            if live_plotting:
                if self.live_plot.should_update(timestamp) and self.queue_len() < 10:
                    self.dummy1.xtz_plot.append(self.dummy1.accel_x, self.dummy1.dt, self.dummy1.accel_z)
                    self.dummy1.xyz_plot.append(self.dummy1.accel_x, self.dummy1.accel_y, self.dummy1.accel_z)

                    if self.dummy1.container_plot.enabled:
                        self.dummy1.xs[self.dummy1.accel_z % 180] = self.dummy1.accel_x
                        self.dummy1.ys[self.dummy1.accel_z % 180] = self.dummy1.accel_y
                    self.dummy1.container_plot.update(self.dummy1.xs, self.dummy1.ys)

                    self.dummy1.time_lag.append(self.dt, timestamp - self.dummy1.dt)
                    self.dummy1.queue_count.append(self.dt, self.queue_len())

                    if not self.live_plot.plot():
                        return False
                else:
                    self.record("time lag", "%7.5f\t%7.5f" % (timestamp, self.queue_len()))
                    # elif self.did_receive(self.dummy2):
                    #     print(self.dummy2.dt_ms, self.dummy2.dt_us)
                    # elif self.did_receive(self.dummy3):
                    #     print(self.dummy3.dt_ms, self.dummy3.dt_us)

    def loop(self):
        if self.joystick is not None:
            if self.joystick.button_updated('A'):
                self.dummy1.set_led('r', self.joystick.get_button('A'))
            if self.joystick.button_updated('B'):
                self.dummy1.set_led('b', 255 * int(self.joystick.get_button('B')))
            if self.joystick.button_updated('X'):
                self.dummy1.set_led('g', self.joystick.get_button('X'))
            if self.joystick.button_updated('Y'):
                self.dummy1.set_led('y', self.joystick.get_button('Y'))

            if self.joystick.button_updated('LB'):
                self.dummy2.set_led(self.joystick.get_button('LB'))
            if self.joystick.button_updated('RB'):
                self.dummy3.set_led(self.joystick.get_button('RB'))

            if self.joystick.axis_updated("right y"):
                self.dummy1.set_led('b', abs(
                    int(self.joystick.get_axis("right y") * 255)))

    def close(self):
        if live_plotting:
            self.live_plot.close()


def run_dummy():
    DummyRunner().run()


run_dummy()
