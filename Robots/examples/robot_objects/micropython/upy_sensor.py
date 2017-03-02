from atlasbuggy.robot.object import RobotObject

# robot objects can have plots associated with them
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection


class Dummy1(RobotObject):
    def __init__(self):
        self.accel_x, self.accel_y, self.accel_z = 0, 0, 0
        self.dt = 0.0
        self.switch = False

        self.python_version = None
        self.micropython_version = None

        self.leds = [False, False, False]
        self.led_names = {
            "r": 0, "g": 1, "y": 2
        }
        self.led_indices = {
            0: "r", 1: "g", 2: "y"
        }
        self.blue_led = 0

        # ----- plot objects -----
        gravity_range = (-90, 90)
        self.xyz_plot = RobotPlot(
            "dummy xyz",
            # plot_enabled=False,
            flat_plot=False, color="red",
            x_range=gravity_range, y_range=gravity_range, z_range=gravity_range
        )
        self.xtz_plot = RobotPlot(
            "dummy xtz",
            # plot_enabled=False,
            flat_plot=False, color="blue",
            x_range=gravity_range, z_range=gravity_range
        )

        self.container_plot = RobotPlot(
            "container",
            # plot_enabled=False,
            flat_plot=True, color="green",
            linestyle="None", marker=".",
            x_range=gravity_range, y_range=gravity_range
        )

        self.time_lag = RobotPlot(
            "time lag",
            # plot_enabled=False
        )
        self.queue_count = RobotPlot(
            "queue count",
            # plot_enabled=False
        )

        self.time_plot = RobotPlotCollection(
            "time plot", self.time_lag, self.queue_count,
            # plot_enabled=False
        )

        self.xs = [0] * 180
        self.ys = [0] * 180

        super(Dummy1, self).__init__("dummy1")

    def receive_first(self, packet):
        data = packet.split("\t")
        self.python_version = data[0]
        self.micropython_version = data[1]

        index = 0
        for led_state in data[2: 2 + len(self.leds)]:
            self.leds[index] = bool(int(led_state))
            index += 1

        self.blue_led = int(data[5])

        print("versions:", self.python_version, self.micropython_version)

    def receive(self, timestamp, packet):
        if packet[0] == "a":
            data = packet[1:].split("\t")
            self.dt = float(data[0])
            self.accel_x = int(data[1])
            self.accel_y = int(data[2])
            self.accel_z = int(data[3])
        elif packet[0] == "s":
            print("switch pressed")

    def set_led(self, color, value):
        if type(color) == int:
            color = self.led_indices[color]

        self.send("%s%i" % (color[0], int(value)))
        if color == "b":
            self.blue_led = value
        else:
            self.leds[self.led_names[color]] = value

    def parse_command(self, packet):
        if packet[0] == "b":
            self.blue_led = int(packet[1:])
        else:
            self.leds[self.led_names[packet[0]]] = bool(int(packet[1:]))
