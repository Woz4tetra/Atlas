from atlasbuggy.robot.object import RobotObject


class Brakes(RobotObject):
    def __init__(self, enabled=True):
        self.position = 0
        self.goal_position = 0

        self.brake_pos = None
        self.unbrake_pos = None

        self.pos_error = None

        self.engaged = False
        self.moving = False

        super(Brakes, self).__init__("linear actuator", enabled)

    def receive_first(self, packet):
        data = packet.split("\t")
        self.brake_pos = int(data[0])
        self.unbrake_pos = int(data[1])

    def receive(self, timestamp, packet):
        self.moving = packet[0] == 'm'
        self.position = int(packet[1:])

    def brake(self):
        self.send('b')
        self.engaged = True

    def unbrake(self):
        self.send('r')
        self.engaged = False

    def toggle(self):
        if self.engaged:
            self.unbrake()
        else:
            self.brake()

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        string += "goal: %2.0d, position: %2.0d\n" % (self.goal_position, self.position)
        return string
