from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self, enabled=True):
        self.update_rate_hz = None
        self.baud_rate = None

        super(GPS, self).__init__("gps", enabled)

    def receive_first(self, packet):
        data = packet.split("\t")
        self.update_rate_hz = int(data[0])
        self.baud_rate = int(data[1])

    def receive(self, timestamp, packet):
        print(packet)
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
                self.fix = bool(subsegments[0])
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
