from atlasbuggy.robot.robotobject import RobotObject


class ImuVector:
    def __init__(self, *vector):
        if len(vector) == 0:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0

        if len(vector) >= 1:
            self.x = vector[0]
        else:
            self.y = 0.0
            self.z = 0.0

        if len(vector) >= 2:
            self.y = vector[1]
        else:
            self.z = 0.0

        if len(vector) >= 3:
            self.z = vector[2]

    def __getitem__(self, item):
        if type(item) == int:
            if item == 0:
                return self.x
            elif item == 1:
                return self.y
            elif item == 2:
                return self.z
        else:
            return self.__dict__[item]

    def __setitem__(self, item, value):
        if type(item) == int:
            if item == 0:
                self.x = value
            elif item == 1:
                self.y = value
            elif item == 2:
                self.z = value
        else:
            self.__dict__[item] = value

    def get_tuple(self):
        return (self.x, self.y, self.z)

class IMU(RobotObject):
    def __init__(self, enabled=True):
        self.sample_rate = None

        self.euler = ImuVector()
        self.accel = ImuVector()
        self.gyro = ImuVector()
        self.mag = ImuVector()
        self.linaccel = ImuVector()

        super(IMU, self).__init__("imu", enabled)

    def receive_first(self, packet):
        header = "delay:"
        self.sample_rate = int(packet[len(header):])

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        for segment in data:
            if len(segment) > 0:
                if segment[0] == "e":
                    self.euler[segment[1]] = float(segment[2:])
                elif segment[0] == "a":
                    self.accel[segment[1]] = float(segment[2:])
                elif segment[0] == "g":
                    self.gyro[segment[1]] = float(segment[2:])
                elif segment[0] == "m":
                    self.mag[segment[1]] = float(segment[2:])
                elif segment[0] == "l":
                    self.linaccel[segment[1]] = float(segment[2:])
                else:
                    print("Invalid segment type!", segment[0], data)
            else:
                print("Empty segment!", data)

    def __str__(self):
        string = "%s(enabled=%s)\n\t" % (self.__class__.__name__, self.enabled)
        string += "euler: (%4.6f, %4.6f, %4.6f)\n\t" % tuple(self.euler.get_tuple())
        string += "accel: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.accel.get_tuple())
        string += "gyro: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.gyro.get_tuple())
        string += "mag: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.gyro.get_tuple())
        string += "linaccel: (%2.6f, %2.6f, %2.6f)\n" % tuple(self.linaccel.get_tuple())
        return string
