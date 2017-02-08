from atlasbuggy.robot.robotobject import RobotObject


class Brakes(RobotObject):
    def __init__(self, enabled=True):
        self.position = 0
        self.goal_position = 0

        self.brake_pos = None
        self.unbrake_pos = None

        self.pos_error = None

        super(Brakes, self).__init__("brakes", enabled)

    def receive_first(self, packet):
        data = packet.split("\t")
        self.pos_error = int(data[0])
        self.brake_pos = int(data[1])
        self.unbrake_pos = int(data[2])

    def receive(self, timestamp, packet):
        self.position = int(packet)
        self.goal_position = self.position

    def brake(self):
        self.set_brake(self.brake_pos)

    def unbrake(self):
        self.set_brake(self.unbrake_pos)

    def set_brake(self, position):
        self.goal_position = position
        print("sending position:", repr(position))
        self.send(position)

    # def is_engaged(self):
    #     return
