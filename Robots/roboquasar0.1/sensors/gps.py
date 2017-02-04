from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self, enabled=True):
        self.update_rate_hz = None
        self.baud_rate = None

        super(GPS, self).__init__("gps", enabled)

    def receive_first(self, packet):
        data = packet.split("\t")
        self.update_rate_hz = int(data[0])
        self.baud_rate = int(data[1])

    def receive(self, timestamp, packet):
        print(">", repr(packet))
