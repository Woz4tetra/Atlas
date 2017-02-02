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

        self.linaccel_x = 0.0
        self.linaccel_y = 0.0
        self.linaccel_z = 0.0

        self.data = [0]*12

        super(IMU, self).__init__("imu", enabled)

    def receive_first(self, packet):
        header = "delay:"
        self.sample_rate = int(packet[len(header)])

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        print(repr(packet))

        self.eul_x = float(data[0][2:])
        self.eul_y = float(data[1][2:])
        self.eul_z = float(data[2][2:])

        self.mag_x = float(data[3][2:])
        self.mag_y = float(data[4][2:])
        self.mag_z = float(data[5][2:])

        self.gyro_x = float(data[6][2:])
        self.gyro_y = float(data[7][2:])
        self.gyro_z = float(data[8][2:])

        self.accel_x = float(data[9][2:])
        self.accel_y = float(data[10][2:])
        self.accel_z = float(data[11][2:])

        self.linaccel_x = float(data[12][3:])
        self.linaccel_y = float(data[13][3:])
        self.linaccel_z = float(data[14][3:])
