from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self):
        super(GPS, self).__init__("gps")

    def receive_first(self, packet):
        pass

    def receive(self, timestamp, packet):
        pass
