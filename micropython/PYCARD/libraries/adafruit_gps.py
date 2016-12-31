import pyb

PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ = "$PMTK220,10000*2F"
PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ = "$PMTK220,5000*1B"  # Once every 5 seconds, 200 millihertz.
PMTK_SET_NMEA_UPDATE_1HZ = "$PMTK220,1000*1F"
PMTK_SET_NMEA_UPDATE_5HZ = "$PMTK220,200*2C"
PMTK_SET_NMEA_UPDATE_10HZ = "$PMTK220,100*2F"
# Position fix update rate commands.
PMTK_API_SET_FIX_CTL_100_MILLIHERTZ = "$PMTK300,10000,0,0,0,0*2C"  # Once every 10 seconds, 100 millihertz.
PMTK_API_SET_FIX_CTL_200_MILLIHERTZ = "$PMTK300,5000,0,0,0,0*18"  # Once every 5 seconds, 200 millihertz.
PMTK_API_SET_FIX_CTL_1HZ = "$PMTK300,1000,0,0,0,0*1C"
PMTK_API_SET_FIX_CTL_5HZ = "$PMTK300,200,0,0,0,0*2F"
# Can't fix position faster than 5 times a second!


PMTK_SET_BAUD_57600 = "$PMTK251,57600*2C"
PMTK_SET_BAUD_9600 = "$PMTK251,9600*17"

# turn on only the second sentence (GPRMC)
PMTK_SET_NMEA_OUTPUT_RMCONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
# turn on GPRMC and GGA
PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
# turn on ALL THE DATA
PMTK_SET_NMEA_OUTPUT_ALLDATA = "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
# turn off output
PMTK_SET_NMEA_OUTPUT_OFF = "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

# to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
# such as the awesome http:#www.hhhh.org/wiml/proj/nmeaxor.html

PMTK_LOCUS_STARTLOG = "$PMTK185,0*22"
PMTK_LOCUS_STOPLOG = "$PMTK185,1*23"
PMTK_LOCUS_STARTSTOPACK = "$PMTK001,185,3*3C"
PMTK_LOCUS_QUERY_STATUS = "$PMTK183*38"
PMTK_LOCUS_ERASE_FLASH = "$PMTK184,1*22"
LOCUS_OVERLAP = 0
LOCUS_FULLSTOP = 1

PMTK_ENABLE_SBAS = "$PMTK313,1*2E"
PMTK_ENABLE_WAAS = "$PMTK301,2*2E"

# standby command & boot successful message
PMTK_STANDBY = "$PMTK161,0*28"
PMTK_STANDBY_SUCCESS = "$PMTK001,161,3*36"  # Not needed currently
PMTK_AWAKE = "$PMTK010,002*2D"

# ask for the release and version
PMTK_Q_RELEASE = "$PMTK605*31"

# request for updates on antenna status
PGCMD_ANTENNA = "$PGCMD,33,1*6C"
PGCMD_NOANTENNA = "$PGCMD,33,0*6D"

# how long to wait when we're looking for a response
MAXWAITSENTENCE = 5
MAXLINELENGTH = 120


class AdafruitGPS:
    def __init__(self, uart_bus, timer_num, baud_rate, update_rate):
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

        self.init_uart(uart_bus, baud_rate, update_rate)
        self.timer = pyb.Timer(timer_num, freq=update_rate)
        self.timer.callback(lambda t: self.timer_callback())

    def init_uart(self, uart_bus, baud_rate, update_rate):
        self.uart = pyb.UART(uart_bus, baud_rate, read_buf_len=1000)

        pyb.delay(50)

        # get RMC and GGA sentences at 1 hz
        if update_rate == 1:
            self.send_command(PMTK_SET_NMEA_OUTPUT_RMCGGA)
            self.send_command(PMTK_SET_NMEA_UPDATE_1HZ)
            self.send_command(PMTK_API_SET_FIX_CTL_1HZ)
        elif update_rate == 5:
            # get RMC and GGA sentences at 5 hz
            self.send_command(PMTK_SET_NMEA_OUTPUT_RMCGGA)
            self.send_command(PMTK_SET_NMEA_UPDATE_5HZ)
            self.send_command(PMTK_API_SET_FIX_CTL_5HZ)
        elif update_rate == 10:
            if baud_rate == 9600:  # send less data if using slower baud rate
                self.send_command(PMTK_SET_NMEA_OUTPUT_RMCONLY)
            elif baud_rate == 57600:
                self.send_command(PMTK_SET_NMEA_OUTPUT_RMCGGA)
            else:
                raise ValueError("Invalid baud rate:", baud_rate)

            self.send_command(PMTK_SET_NMEA_UPDATE_10HZ)
            # fix can't update at 10 hz
            self.send_command(PMTK_API_SET_FIX_CTL_5HZ)
        else:
            raise ValueError("Invalid update rate:", update_rate)

    def timer_callback(self):
        self.recved_flag = True

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
        """
        example:
        $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
        
        VTG          Track made good and ground speed
        054.7,T      True track made good (degrees)
        034.4,M      Magnetic track made good
        005.5,N      Ground speed, knots
        010.2,K      Ground speed, Kilometers per hour
        *48          Checksum
        """
        track_angle_true, _, track_angle_magnetic, _, speed_knots, _, \
        speed_kmph, _, _ = self.parse_sentence(sentence, 9)

        self.track_angle_true = track_angle_true
        self.track_angle_magnetic = track_angle_magnetic

        self.speed_knots = speed_knots
        self.speed_kmph = speed_kmph

    def parse_gsa_sentence(self, sentence):
        """
        example:
        $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

        GSA      Satellite status
        A        Auto selection of 2D or 3D fix (M = manual) 
        3        3D fix - values include: 1 = no fix
                                          2 = 2D fix
                                          3 = 3D fix
        04,05... PRNs of satellites used for fix (space for 12) 
        2.5      PDOP (dilution of precision) 
        1.3      Horizontal dilution of precision (HDOP) 
        2.1      Vertical dilution of precision (VDOP)
        *39      the checksum data, always begins with *
        """
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
        """
        example:
        $GPGSV,2,1,08,01,40,083,46,02,17,308,41,12,07,344,39,14,22,228,45*75
        
        GSV          Satellites in view
        2            Number of sentences for full data
        1            sentence 1 of 2
        08           Number of satellites in view

        01           Satellite PRN number
        40           Elevation, degrees
        083          Azimuth, degrees
        46           SNR - higher is better for up to 4 satellites per sentence
        *75          the checksum data, always begins with *
        """

        pass  # not tracking each satellite's info

    def parse_gga_sentence(self, sentence):
        """
        example:
        $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

        GGA          Global Positioning System Fix Data
        123519       Fix taken at 12:35:19 UTC
        4807.038,N   Latitude 48 deg 07.038' N
        01131.000,E  Longitude 11 deg 31.000' E
        1            Fix quality: 0 = invalid
                                  1 = GPS fix (SPS)
                                  2 = DGPS fix
                                  3 = PPS fix
                                  4 = Real Time Kinematic
                                  5 = Float RTK
                                  6 = estimated (dead reckoning) (2.3 feature)
                                  7 = Manual input mode
                                  8 = Simulation mode
        08           Number of satellites being tracked
        0.9          Horizontal dilution of position
        545.4,M      Altitude, Meters, above mean sea level
        46.9,M       Height of geoid (mean sea level) above WGS84
                         ellipsoid
        (empty field) time in seconds since last DGPS update
        (empty field) DGPS station ID number
        *47          the checksum data, always begins with *

        see http://www.gpsinformation.org/dale/nmea.htm#GGA for more
        """

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
        """
        example:
        $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

         RMC          Recommended Minimum sentence C
         123519       Fix taken at 12:35:19 UTC
         A            Status A=active or V=Void.
         4807.038,N   Latitude 48 deg 07.038' N
         01131.000,E  Longitude 11 deg 31.000' E
         022.4        Speed over the ground in knots
         084.4        Track angle in degrees True
         230394       Date - 23rd of March 1994
         003.1,W      Magnetic Variation
         *6A          The checksum data, always begins with *
        """
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

    def parse_date(self, date_sentence):
        if len(date_sentence) >= 6:
            self.day = int(date_sentence[0:2])
            self.month = int(date_sentence[2:4])
            self.year = int(date_sentence[4:6])

    def parse_time(self, time_sentence):
        if len(time_sentence) >= 6:
            time = float(time_sentence)
            self.hour = int(time / 10000)
            self.minute = int((time % 10000) / 100)
            self.seconds = int(time % 100)
            self.milliseconds = (time % 1.0) * 1000 

    def parse_latitude(self, latitude, lat_direction):
        if len(latitude) >= 4:
            self.latitude_deg = int(latitude[0:2])
            self.latitude_min = float(latitude[2:])
            self.latitude = self.latitude_deg + self.latitude_min / 60
        if len(lat_direction) > 0:
            self.lat_direction = lat_direction

    def parse_longitude(self, longitude, long_direction):
        if len(longitude) >= 4:
            self.longitude_deg = int(longitude[0:3])
            self.longitude_min = float(longitude[3:])
            self.longitude = -self.longitude_deg - self.longitude_min / 60
        if len(long_direction) > 0:
            self.long_direction = long_direction

    @staticmethod
    def parse_hex(char):
        # convert hex byte to int
        return int(char, 16)
    
    def read_sentences(self):
        read_num = self.uart.any()
        if read_num > 30:
            read_num = 30
        self.buffer += self.uart.read(read_num)
        
        sentences = self.buffer.split(b'\n')
        if self.buffer[-1] != '\n':# or self.buffer[-1] != ' ':
            self.buffer = sentences.pop(-1)
        else:
            self.buffer = b''
        return sentences

    def received_sentence(self):
        if self.recved_flag is True:
            self.recved_flag = False

            if self.uart.any():
                self.previous_sentence = self.sentence
                valid_sentence = False
#                for sentence in self.read_sentences():
                self.sentence = self.uart.readline() #sentence
                valid_sentence = self.parse(self.sentence)
                
                return valid_sentence
        return False

    def wait_for_sentence(self, sentence, max_wait_count=5):
        counter = 0
        while counter < max_wait_count:
            if self.received_sentence():
                # current_line is modified constantly. previous_line is the
                # last complete sentence
                if sentence == self.previous_sentence:
                    return True
                counter += 1
        return False

    def standby(self):
        if self.in_standby_mode:  # do nothing if in standby already
            return False
        else:
            self.in_standby_mode = True
            self.send_command(PMTK_STANDBY)
            return True

    def wakeup(self):
        if self.in_standby_mode:
            self.in_standby_mode = True
            self.send_command("")  # send byte (newline) to wake it up
            return self.wait_for_sentence(PMTK_AWAKE)
        else:
            return False

    def send_command(self, command):  # command is a string
        self.uart.write(command.encode('ascii') + b'\n')
