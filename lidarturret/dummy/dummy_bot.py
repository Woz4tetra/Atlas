import time

from atlasbuggy.interface import RobotObject


class Dummy(RobotObject):
    def __init__(self):
        self.accel_x, self.accel_y, self.accel_z = 0, 0, 0
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

        self.time0 = time.time()

        super(Dummy, self).__init__("dummy")

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

    def receive(self, packet):
        data = packet.split("\t")
        self.accel_x = int(data[0])
        self.accel_y = int(data[1])
        self.accel_z = int(data[2])

    def set_led(self, color, value):
        if type(color) == int:
            color = self.led_indices[color]

        self.send("%s%i" % (color[0], int(value)))
        if color == "b":
            self.blue_led = value
        else:
            self.leds[self.led_names[color]] = value
