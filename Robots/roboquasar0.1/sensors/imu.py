from atlasbuggy.robot.robotobject import RobotObject


class IMU(RobotObject):
    def __init__(self, enabled=True):
        self.sample_rate = None
        self.eul_x = 0.0
        self.eul_y = 0.0
        self.eul_z = 0.0

        self.accel_x = 0.0
        self.accel_y = 0.0
        self.accel_z = 0.0

        self.mag_x = 0.0
        self.mag_y = 0.0
        self.mag_z = 0.0

        self.gyro_x = 0.0
        self.gyro_y = 0.0
        self.gyro_z = 0.0

        super(IMU, self).__init__("imu", enabled)

    def receive_first(self, packet):
        header = "delay:"
        self.sample_rate = int(packet[len(header)])

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        self.eul_x = float(data[0])
        self.eul_y = float(data[1])
        self.eul_z = float(data[2])

        self.mag_x = float(data[3])
        self.mag_y = float(data[4])
        self.mag_z = float(data[5])

        self.gyro_x = float(data[6])
        self.gyro_y = float(data[7])
        self.gyro_z = float(data[8])

        self.accel_x = float(data[9])
        self.accel_y = float(data[10])
        self.accel_z = float(data[11])
