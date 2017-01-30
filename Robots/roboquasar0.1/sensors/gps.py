from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self, enabled=True):
        self.hour = 0
        self.minute = 0
        self.second = 0
        self.milliseconds = 0

        self.day = 0
        self.month = 0
        self.year = 0

        self.fix_quality = 0

        self.latitude = 0
        self.lat = 0
        self.longitude = 0
        self.lon = 0
        self.latitude_degree = 0
        self.longitude_degree = 0

        self.knots = 0
        self.angle = 0
        self.altitude = 0
        self.satellites = 0

        super(GPS, self).__init__("gps", enabled)

    def receive_first(self, packet):
        # this shouldn't be called
        pass

    def receive(self, timestamp, packet):
        data = packet.split("\t")

        # time
        self.hour = float(data[0])
        self.minute = float(data[1])
        self.second = float(data[2])
        self.milliseconds = float(data[3])

        # date
        self.day = float(data[4])
        self.month = float(data[5])
        self.year = float(data[6])

        # fix info
        self.fix_quality = float(data[7])

        # location info
        self.latitude = float(data[8])
        self.lat = float(data[9])
        self.longitude = float(data[10])
        self.lon = float(data[11])
        self.latitude_degree = float(data[12])
        self.longitude_degree = float(data[13])

        # cool shit
        self.knots = float(data[14])
        self.angle = float(data[15])
        self.altitude = float(data[16])
        self.satellites = float(data[17])
