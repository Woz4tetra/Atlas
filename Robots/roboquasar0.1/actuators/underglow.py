from atlasbuggy.robot.robotobject import RobotObject
from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection


class Underglow(RobotObject):
    def __init__(self, enabled=True):
        self.num_leds = 30
        self.leds = [(255, 255, 255) for _ in range(self.num_leds)]

        self.led_plots = [
            RobotPlot("LED #%s" % x, marker='.', markersize=10, x_range=(-1, self.num_leds), color='black') for x in
            range(self.num_leds)]
        self.strip_plot = RobotPlotCollection("LEDs", *self.led_plots)

        super(Underglow, self).__init__("underglow", enabled)

    def set_led(self, led_num, r, g, b):
        if r > 255:
            r = 255
        if g > 255:
            g = 255
        if b > 255:
            b = 255

        r = int(abs(r))
        g = int(abs(g))
        b = int(abs(b))
        led_num = int(abs(led_num))

        self.send("l%3.0d%3.0d%3.0d%3.0d" % (led_num, r, g, b))
        print("l%3.0d%3.0d%3.0d%3.0d" % (led_num, r, g, b))
        print("#%2.0d\tr: %3.0d\tg: %3.0d\tb: %3.0d" % (led_num, r, g, b))

        if self.strip_plot is not None and self.strip_plot.enabled:
            self.led_plots[led_num].set_properties(color=(r / 255, g / 255, b / 255))
            self.leds[led_num] = (r, g, b)
