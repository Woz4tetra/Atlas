from atlasbuggy.robot.object import RobotObject
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection


class Underglow(RobotObject):
    def __init__(self, enabled=True, enable_plotting=False):
        self.enable_plotting = enable_plotting
        self.num_leds = None
        self.leds = None
        self.led_plots = None
        self.strip_plot = None

        self.brake_signal_cycles = 4
        self.brake_signal_r = 255
        self.brake_signal_g = 255
        self.brake_signal_b = 0

        self.release_signal_cycles = 4
        self.release_signal_r = 0
        self.release_signal_g = 255
        self.release_signal_b = 255

        super(Underglow, self).__init__("underglow", enabled)

    def receive_first(self, packet):
        self.num_leds = int(packet)
        self.leds = [(255, 255, 255) for _ in range(self.num_leds)]

        self.led_plots = [
            RobotPlot("LED #%s" % x, enabled=self.enable_plotting, marker='.', markersize=10,
                      x_range=(-1, self.num_leds), color='black') for x in
            range(self.num_leds)]
        self.strip_plot = RobotPlotCollection("LEDs", *self.led_plots, enabled=self.enable_plotting)

    def receive(self, timestamp, packet):
        # print(timestamp, repr(packet))
        pass

    def constrain_input(self, rgb):
        if len(rgb) == 1:
            rgb = rgb[0]
        r = int(abs(rgb[0]))
        g = int(abs(rgb[1]))
        b = int(abs(rgb[2]))

        if r > 255:
            r = 255
        if g > 255:
            g = 255
        if b > 255:
            b = 255

        return r, g, b

    def set_leds(self, start, end, *rgb):
        r, g, b = self.constrain_input(rgb)
        start = int(abs(start))
        end = int(abs(end))

        assert end <= self.num_leds and 0 <= start and start < end

        self.send("l%3.0d%3.0d%3.0d%3.0d%3.0d" % (start, r, g, b, end))
        # print("l%3.0d%3.0d%3.0d%3.0d%3.0d" % (start, r, g, b, end))
        # print("#%2.0d\tr: %3.0d\tg: %3.0d\tb: %3.0d" % (led_num, r, g, b))

        if self.strip_plot.enabled:
            for led_num in range(start, end):
                self.led_plots[led_num].set_properties(color=(r / 255, g / 255, b / 255))
                self.leds[led_num] = (r, g, b)

    def set_led(self, led_num, *rgb):
        r, g, b = self.constrain_input(rgb)
        led_num = int(abs(led_num))

        self.send("l%03d%03d%03d%03d" % (led_num, r, g, b))
        # print("l%3.0d%3.0d%3.0d%3.0d" % (led_num, r, g, b))
        # print("#%2.0d\tr: %3.0d\tg: %3.0d\tb: %3.0d" % (led_num, r, g, b))

        if self.strip_plot.enabled:
            self.led_plots[led_num].set_properties(color=(r / 255, g / 255, b / 255))
            self.leds[led_num] = (r, g, b)

    def signal_brake(self):
        self.send(
            "f%03d%03d%03d%03d" % (
                self.brake_signal_cycles, self.brake_signal_r, self.brake_signal_g, self.brake_signal_b)
        )

    def signal_release(self):
        self.send(
            "f%03d%03d%03d%03d" % (
                self.release_signal_cycles, self.release_signal_r, self.release_signal_g, self.release_signal_b)
        )

    def rainbow_cycle(self):
        self.send("r")

    def color_wipe(self, delay, *rgb):
        r, g, b = self.constrain_input(rgb)
        self.send("wipe%03d%03d%03d%03d" % (r, g, b, delay))

    def fancy_gradient(self, start):
        self.send("g%02d" % start)

    def set_all(self, *rgb):
        self.set_leds(0, self.num_leds, *rgb)

    def show(self):
        self.send("d")

    def __len__(self):
        return self.num_leds
