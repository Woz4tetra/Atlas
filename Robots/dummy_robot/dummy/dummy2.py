from atlasbuggy.robot.robotobject import RobotObject


class Dummy2(RobotObject):
    def __init__(self):
        self.dt_ms = 0.0
        self.dt_us = 0.0

        self.dummy_data = None
        self.led_state = False

        super(Dummy2, self).__init__("dummy2")

    def receive_first(self, packet):
        data = packet.split("\t")
        self.dummy_data = int(data[0])
        self.led_state = bool(int(data[1]))

        print("init:", self.dummy_data, self.led_state)

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        self.dt_ms = int(data[0])
        self.dt_us = int(data[1])

    def set_led(self, value):
        self.send("l%i" % (int(value)))
