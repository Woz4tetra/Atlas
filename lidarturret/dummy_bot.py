import time

from robot.interface import RobotObject


class Dummy(RobotObject):
    def __init__(self):
        self.accel_x, self.accel_y, self.accel_z = 0, 0, 0
        self.switch = False

        self.python_version = None
        self.micropython_version = None

        self.leds = [False, False, False]
        self.led_names = {
            "red": 0, "green": 1, "yellow": 2
        }
        self.led_indices = {
            0: "red", 1: "green", 2: "yellow"
        }
        self.blue_led = 0

        self.time0 = time.time()

        super(Dummy, self).__init__("dummy", "/dev/tty.usbmodem*")

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
        if color == "blue":
            self.blue_led = value
        else:
            self.leds[self.led_names[color]] = value
