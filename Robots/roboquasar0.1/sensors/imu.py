import math
from atlasbuggy.robot.object import RobotObject


class ImuVector:
    def __init__(self, *vector):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        if len(vector) >= 1:
            self.x = vector[0]
        if len(vector) >= 2:
            self.y = vector[1]
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
        return self.x, self.y, self.z


class QuatVector:
    def __init__(self, *vector):
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        if len(vector) >= 1:
            self.w = vector[0]
        if len(vector) >= 2:
            self.x = vector[1]
        if len(vector) >= 3:
            self.y = vector[2]
        if len(vector) >= 4:
            self.z = vector[3]

    def __getitem__(self, item):
        if type(item) == int:
            if item == 0:
                return self.w
            elif item == 1:
                return self.x
            elif item == 2:
                return self.y
            elif item == 3:
                return self.z
        else:
            return self.__dict__[item]

    def __setitem__(self, item, value):
        if type(item) == int:
            if item == 0:
                self.w = value
            elif item == 1:
                self.x = value
            elif item == 2:
                self.y = value
            elif item == 3:
                self.z = value
        else:
            self.__dict__[item] = value

    def get_tuple(self):
        return self.w, self.x, self.y, self.z


class IMU(RobotObject):
    def __init__(self, enabled=True):
        self.sample_rate = None

        self.euler = ImuVector()
        self.accel = ImuVector()
        self.gyro = ImuVector()
        self.mag = ImuVector()
        self.linaccel = ImuVector()
        self.quat = QuatVector()

        self.system_status = 0
        self.accel_status = 0
        self.gyro_status = 0
        self.mag_status = 0

        super(IMU, self).__init__("imu", enabled)

    def receive_first(self, packet):
        header = "delay:"
        self.sample_rate = int(packet[len(header):])

    def receive(self, timestamp, packet):
        data = packet.split("\t")
        try:
            for segment in data:
                if len(segment) > 0:
                    if segment[0] == "e":
                        self.euler[segment[1]] = math.radians(float(segment[2:]))
                    elif segment[0] == "a":
                        self.accel[segment[1]] = float(segment[2:])
                    elif segment[0] == "g":
                        self.gyro[segment[1]] = float(segment[2:])
                    elif segment[0] == "m":
                        self.mag[segment[1]] = float(segment[2:])
                    elif segment[0] == "l":
                        self.linaccel[segment[1]] = float(segment[2:])
                    elif segment[0] == "q":
                        self.quat[segment[1]] = float(segment[2:])
                    elif segment[0] == "s":
                        if segment[1] == "s":
                            self.system_status = int(segment[2:])
                        elif segment[1] == "a":
                            self.accel_status = int(segment[2:])
                        elif segment[1] == "g":
                            self.gyro_status = int(segment[2:])
                        elif segment[1] == "m":
                            self.mag_status = int(segment[2:])
                    else:
                        print("IMU: Invalid segment type!", segment[0], data)
                else:
                    print("IMU: Empty segment!", data)
        except ValueError:
            print("Failed to parse:", segment)

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)

        string += "euler: (%4.6f, %4.6f, %4.6f)\n\t" % tuple(self.euler.get_tuple())
        string += "accel: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.accel.get_tuple())
        string += "gyro: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.gyro.get_tuple())
        string += "mag: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.gyro.get_tuple())
        string += "linaccel: (%2.6f, %2.6f, %2.6f)\n\t" % tuple(self.linaccel.get_tuple())
        string += "quat: (%2.6f, %2.6f, %2.6f, %2.6f)\n\t" % tuple(self.quat.get_tuple())
        string += "system: %s, accel: %s, gyro: %s, mag: %s\n" % (
            self.system_status, self.accel_status, self.gyro_status, self.mag_status)
        return string
