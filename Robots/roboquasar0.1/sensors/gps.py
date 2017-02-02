from atlasbuggy.robot.robotobject import RobotObject


class GPS(RobotObject):
    def __init__(self, enabled=True):
        self.hour = 0
        self.minute = 0
        self.seconds = 0
        self.milliseconds = 0.0

        self.day = 0
        self.month = 0
        self.year = 0

        self.latitude_deg = 0
        self.latitude_min = 0.0
        self.latitude = 0.0
        self.lat_direction = 'N'

        self.longitude_deg = 0
        self.longitude_min = 0.0
        self.longitude = 0.0
        self.long_direction = 'W'

        self.num_satellites = 0
        self.pdop, self.hdop, self.vdop = 0.0, 0.0, 0.0

        self.altitude = 0.0
        self.geoid_height = 0.0

        self.fix = False

        self.speed_knots = 0.0
        self.speed_kmph = 0.0

        self.magnetic_variation = 0.0
        self.mag_var_direction = 'W'

        self.track_angle_true = 0.0
        self.track_angle_magnetic = 0.0

        self.paused = False

        self.recved_flag = True

        self.buffer = b''
        self.sentence = b''
        self.previous_sentence = b''

        self.in_standby_mode = False

        super(GPS, self).__init__("gps", enabled)


    def parse(self, sentence):
        # do checksum, first look if we have one
        if len(sentence) > 6:
            sentence = sentence[:-2].decode('ascii')
            if sentence[-3] == b'*':
                sum = self.parse_hex(sentence[-2]) * 16
                sum += self.parse_hex(sentence[-1])

                # check checksum
                for index in range(1, len(sentence) - 3):
                    sum ^= ord(sentence[index])

                if sum != 0:
                    # bad checksum :(
                    return False
            packet_type = sentence[1:6]

            try:
                if packet_type == "GPGGA":
                    self.parse_gga_sentence(sentence)
                elif packet_type == "GPRMC":
                    self.parse_rmc_sentence(sentence)
                elif packet_type == "GPVTG":
                    self.parse_vtg_sentence(sentence)
                elif packet_type == "GPGSA":
                    self.parse_gsa_sentence(sentence)
                elif packet_type == "GPGSV":
                    # self.parse_gsv_sentence(sentence)
                    pass  # not tracking each satellite's info
                else:
                    print("Unrecognized packet: '%s', '%s'" % (packet_type, sentence))
            except:
                print("Exception, unrecognized packet: '%s', '%s'" % (packet_type, sentence))

            return True
        else:
            return False

    def parse_sentence(self, sentence, expected_num):
        # remove packet type and checksum
        split = tuple(sentence[7:-3].split(","))

        if len(split) != expected_num:
            print("Mismatched parameter number: '%s'\n" % sentence, split)
            if len(split) < expected_num:  # fill missing with None
                return split + ('',) * (expected_num - len(split))
            else:
                return split[0:expected_num]
        else:
            return split

    def parse_vtg_sentence(self, sentence):
        track_angle_true, _, track_angle_magnetic, _, speed_knots, _, \
        speed_kmph, _, _ = self.parse_sentence(sentence, 9)

        self.track_angle_true = track_angle_true
        self.track_angle_magnetic = track_angle_magnetic

        self.speed_knots = speed_knots
        self.speed_kmph = speed_kmph

    def parse_gsa_sentence(self, sentence):
        parsed = self.parse_sentence(sentence, 17)
        # fix_selection, fix_3d = parsed[0:2]
        # satellite_prns = parsed[2:14]  # not tracking each satellite's info
        pdop, hdop, vdop = parsed[14:17]
        if len(pdop) > 0:
            self.pdop = float(pdop)
        if len(hdop) > 0:
            self.hdop = float(hdop)
        if len(vdop) > 0:
            self.vdop = float(vdop)

    def parse_gsv_sentence(self, sentence):
        pass  # not tracking each satellite's info

    def parse_gga_sentence(self, sentence):

        time, latitude, lat_direction, longitude, long_direction, \
        fix_quality, num_satellites, hdop, altitude, altitude_units, \
        geoid_height, geoid_height_units, _, _ = \
            self.parse_sentence(sentence, 14)

        if len(fix_quality) > 0:
            self.fix = int(fix_quality) != 0

        self.parse_time(time)

        self.parse_latitude(latitude, lat_direction)
        self.parse_longitude(longitude, long_direction)

        if len(num_satellites) > 0:
            self.num_satellites = int(num_satellites)
        if len(hdop) > 0:
            self.hdop = float(hdop)

        if len(altitude) > 0:
            self.altitude = float(altitude)
        if len(geoid_height) > 0:
            self.geoid_height = float(geoid_height)

    def parse_rmc_sentence(self, sentence):

        time, fix_status, latitude, lat_direction, longitude, \
        long_direction, speed_knots, bearing, date, _, mag_variation, \
        mag_direction = self.parse_sentence(sentence, 12)

        self.parse_time(time)

        self.fix = True if fix_status == "A" else False

        self.parse_latitude(latitude, lat_direction)
        self.parse_longitude(longitude, long_direction)

        if len(speed_knots) > 0:
            self.speed_knots = float(speed_knots)

        self.parse_date(date)

        if len(mag_variation) > 0:
            self.magnetic_variation = float(mag_variation)
        if len(mag_direction) > 0:
            self.mag_var_direction = mag_direction


    def receive_first(self, packet):
        header = "update delay:"
        self.gps_update_delay = int(packet[len(header):])
        print("update rate: ", 1000 / self.gps_update_delay)

    def receive(self, timestamp, packet):
        parse(packet)
