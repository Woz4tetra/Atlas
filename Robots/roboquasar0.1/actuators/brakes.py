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

        super(Brakes, self).__init__("brakes", enabled)

    def receive_first(self, packet):
        data = packet.split("\t")
        self.brake_pos = int(data[0])
        self.unbrake_pos = int(data[1])

    def receive(self, timestamp, packet):
        try:
            self.moving = packet[0] == 'm'
            self.position = int(packet[1:])
        except IndexError:
            print("BRAKES ERROR PACKET:", timestamp, packet)

    def pull(self):
        self.send('b')
        self.goal_position = self.brake_pos
        self.engaged = True

    def release(self):
        self.send('r')
        self.goal_position = self.unbrake_pos
        self.engaged = False

    def pause_pings(self):
        self.send("p")

    def unpause_pings(self):
        self.send("u")

    def ping(self):
        self.send("t")

    def toggle(self):
        if self.engaged:
            self.release()
        else:
            self.pull()

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        string += "goal: %2.0d, position: %2.0d\n" % (self.goal_position, self.position)
        return string
