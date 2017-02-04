from atlasbuggy.robot.robotobject import RobotObject


class Brakes(RobotObject):
    def __init__(self, enabled=True):
        self.position = 0

        self.brake_pos = 0
        self.unbrake_pos = 255

        self.pos_error = None

        super(Brakes, self).__init__("brakes", enabled)

    def receive_first(self, packet):
        self.pos_error = int(packet)

    def receive(self, timestamp, packet):
        self.position = int(packet)

    def brake(self):
        self.send(self.brake_pos)

    def unbrake(self):
        self.send(self.unbrake_pos)

    def is_engaged(self):
        return
