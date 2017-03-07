from atlasbuggy.robot.object import RobotObject


class AdafruitGPS(RobotObject):
    def __init__(self, enabled=True, enable_limits=True):
        self.update_rate_hz = None
        self.baud_rate = None

        self.hour = 0
        self.minute = 0
        self.seconds = 0
        self.milliseconds = 0

        self.day = 0
        self.minute = 0
        self.year = 0
        self.fix = False
        self.fix_quality = 0

        self.latitude = None
        self.longitude = None

        self.latitude_deg = None
        self.longitude_deg = None

        self.speed = 0
        self.bearing = 0
        self.altitude = 0.0
        self.satellites = 0

        self.min_lat = 40.4
        self.max_lat = 41
        self.min_lon = -80
        self.max_lon = -79.8
        self.min_alt = 280
        self.max_alt = 310

        self.enable_limits = enable_limits

        super(AdafruitGPS, self).__init__("adafruit gps", enabled)
        self.baud = 9600

    def receive_first(self, packet):
        data = packet.split("\t")
        self.update_rate_hz = int(data[0])
        self.baud_rate = int(data[1])

    def receive(self, timestamp, packet):
        # when writing parse methods, it's best to keep things flexible in case an unexpected appears.
        # If any unexpected data comes in (ValueErrors, IndexErrors, etc.) the whole program will stop
        data = packet.split("\t")
        for segment in data:
            if segment[0] == 't':
                subsegments = segment[1:].split(":")
                self.hour = int(subsegments[0])
                self.minute = int(subsegments[1])
                self.seconds = int(subsegments[2])
                self.milliseconds = int(subsegments[3])

            elif segment[0] == 'd':
                subsegments = segment[1:].split("/")
                self.day = int(subsegments[0])
                self.minute = int(subsegments[1])
                self.year = int(subsegments[2])

            elif segment[0] == 'f':
                subsegments = segment[1:].split(",")
                self.fix = bool(int(subsegments[0]))
                self.fix_quality = int(subsegments[1])

            elif segment[0] == 'l':
                subsegments = segment[1:].split(",")
                self.latitude = (float(subsegments[0]), subsegments[1])
                self.longitude = (float(subsegments[2]), subsegments[3])

            elif segment[0] == 'g':
                subsegments = segment[1:].split(",")
                self.latitude_deg = float(subsegments[0])
                self.longitude_deg = float(subsegments[1])

            elif segment[0] == 'x':
                subsegments = segment[1:].split(",")
                self.speed = float(subsegments[0])
                self.bearing = float(subsegments[1])
                self.altitude = float(subsegments[2])
                self.satellites = int(subsegments[3])

    def is_position_valid(self):
        if not self.enable_limits:
            return True

        if not self.fix or self.latitude_deg is None or self.longitude_deg is None or self.altitude is None:
            return False

        if not (self.min_alt < self.altitude < self.max_alt) or self.altitude == 0.0:
            self.altitude = 300.0

        if (self.min_lon < self.longitude_deg < self.max_lon and
                        self.min_lat < self.latitude_deg < self.max_lat and
                        self.min_alt < self.altitude < self.max_alt):
            return True
        else:
            return False

    def __str__(self):
        string = "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
        if self.fix:
            string += "lla: (%4.6f%s, %4.6f%s, %4.6f)\n\t" % (self.latitude[0], self.latitude[1],
                                                              self.longitude[0], self.longitude[1], self.altitude)
            string += "lla degrees: (%2.6f, %2.6f)\n\t" % (self.latitude_deg, self.longitude_deg)
            string += "speed: %2.6f, bearing: %2.6f\n\t" % (self.speed, self.bearing)

        string += "fix: %s, quality: %i, satellites: %i\n\t" % (self.fix, self.fix_quality, self.satellites)
        string += "speed: %2.6f, bearing: %2.6f\n\t" % (self.speed, self.bearing)
        string += "time: %s:%s.%s %s/%s/%s\n" % (self.hour, self.minute, self.seconds, self.day, self.minute, self.year)
        return string
