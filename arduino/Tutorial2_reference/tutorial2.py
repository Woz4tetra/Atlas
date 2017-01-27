import re
from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self):
        self.gps_pattern = re.compile("""
            [a-zA-Z]*:\s*
            (?P<hour>\d*):(?P<minute>\d*):(?P<second>\d*).(?P<millisecond>\d*)\s
            [a-zA-Z]*:\s(?P<month>\d*)/(?P<day>\d*)/(?P<year>\d*)\s
            [a-zA-Z]*:\s(?P<fix>\d*)\s[a-zA-Z]*:\s(?P<quality>\d*)\s
            [a-zA-Z]*:\s(?P<latitude>[-\d.]*)(?P<latdir>[a-zA-Z]*)[, ]*
            (?P<longitude>[-\d.]*)(?P<longdir>[a-zA-Z]*)\s
            [a-zA-Z() :]*(?P<latdeg>[-\d.]*)[, ]*
            (?P<longdeg>[-\d.]*)\s
            [a-zA-Z() :]*(?P<speedknots>[-\d.]*)\s
            [a-zA-Z() :]*(?P<angle>[-\d.]*)\s
            [a-zA-Z() :]*(?P<altitude>[-\d.]*)\s
            [a-zA-Z() :]*(?P<satellites>\d*)
         """, re.VERBOSE)

        self.gps_pattern_short = re.compile("""
            [a-zA-Z]*:\s*
            (?P<hour>\d*):(?P<minute>\d*):(?P<second>\d*).(?P<millisecond>\d*)\s
            [a-zA-Z]*:\s(?P<month>\d*)/(?P<day>\d*)/(?P<year>\d*)\s
            [a-zA-Z]*:\s(?P<fix>\d*)\s[a-zA-Z]*:\s(?P<quality>\d*)""", re.VERBOSE)

        self.init_pattern = re.compile("""
                    [a-zA-Z:]*:(?P<delay>\d*)
                 """, re.VERBOSE)

        self.update_rate = 0

        self.hour = 0
        self.minute = 0
        self.second = 0
        self.millisecond = 0
        self.month = 0
        self.day = 0
        self.year = 0
        self.fix = 0
        self.quality = 0
        self.latitude = 0.0
        self.latdir = 1
        self.longitude = 0.0
        self.longdir = 1
        self.latdeg = 0.0
        self.longdeg = 0.0
        self.speed = 0.0
        self.angle = 0.0
        self.altitude = 0.0
        self.satellites = 0

        super(GPS, self).__init__("gps")

    def receive_first(self, packet):
        match = self.init_pattern.match(packet)
        self.update_rate = 1000 / int(match.group("delay"))

    def receive(self, timestamp, packet):
        count = packet.count("\t")
        if count == 2:
            match = self.gps_pattern_short.match(packet)
        else:
            match = self.gps_pattern.match(packet)

        self.hour = int(match.group("hour"))
        self.minute = int(match.group("minute"))
        self.second = int(match.group("second"))
        self.millisecond = int(match.group("millisecond"))
        self.month = int(match.group("month"))
        self.day = int(match.group("day"))
        self.year = int(match.group("year"))
        self.fix = int(match.group("fix"))
        self.quality = int(match.group("quality"))

        print(self.hour, self.minute, self.second, self.millisecond)

        if count > 2:
            self.latitude = float(match.group("latitude"))
            self.latdir = 1 if match.group("latdir") == "N" else -1
            self.longitude = float(match.group("longitude"))
            self.longdir = 1 if match.group("longdir") == "E" else -1
            self.latdeg = float(match.group("latdeg"))
            self.longdeg = float(match.group("longdeg"))
            self.speed = float(match.group("speedknots"))
            self.angle = float(match.group("angle"))
            self.altitude = float(match.group("altitude"))
            self.satellites = int(match.group("satellites"))

            print(self.latitude, self.longitude)
