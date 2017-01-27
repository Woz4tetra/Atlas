from atlasbuggy.robot.robotobject import RobotObject


class IMU(RobotObject):
    def __init__(self):
        self.sample_rate = None
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        super(IMU, self).__init__("imu")

    def receive_first(self, packet):
        header = "delay:"
        self.sample_rate = int(packet[len(header)])

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        self.x = float(data[0])
        self.y = float(data[1])
        self.z = float(data[2])
