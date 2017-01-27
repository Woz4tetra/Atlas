from atlasbuggy.robot.robotobject import RobotObject


class IMU(RobotObject):
    def __init__(self):
        super(IMU, self).__init__("imu")

    def receive_first(self, packet):
        pass

    def receive(self, timestamp, packet):
        pass
