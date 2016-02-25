"""
    Written by Ben Warwick

    data.py, written for RoboQuasar1.0
    Version 12/28/2015
    =========

    Handles sensor data sorting, the command queue, and raw data storage.

    Classes:
        SensorPool - parses and sorts the incoming packet by giving it to the correct sensor
        CommandQueue - A queue of Command objects. Use put to enqueue a command
        SerialObject - Internal. The superclass of Sensor and Command.
        Sensor - contains the sensor id, expected data format, and parsed sensor data
        Command - contains the command id, data format, and last sent data

    Functions:
        try_sensor - given a list of formats and a hex string, this function
            allows you to test what the output of your sensor would be
            (try with python -i data.py)
        try_command - given a single format (commands are single format)
            and a python built-in type, this functions returns the packet that
            would be written to serial
"""

import struct
import string
import random
import math
import time
from collections import OrderedDict
from sys import maxsize as MAXINT


class SensorPool(object):
    def __init__(self):
        self.sensors = {}

    def add_sensor(self, sensor):
        # add sensors to self.sensors with non-repeating sensor IDs:
        if sensor.object_id in list(self.sensors.keys()):
            raise Exception(
                    "Sensor ID already taken: " + str(sensor.object_id))
        else:
            self.sensors[sensor.object_id] = sensor

    def is_packet(self, packet):
        """
        makes sure the packet is the right format and was safely delivered

        :param packet:
        :return:
        """

        if len(packet) < 5:
            return False
        for c in packet:
            if c.lower() not in '0123456789abcedfiu\t':
                return False
        if packet[2] != '\t' and packet[3] not in 'fibu':
            return False

        return True

    def update(self, packet):
        """
        updates the sensors' data when called and. if correct, parses and replaces the old sensor data

        :param packet:
        :return:
        """

        packet = packet.decode('ascii')
        if self.is_packet(packet):
            sensor_id, data = int(packet[0:2], 16), packet[3:]
            if sensor_id in list(self.sensors.keys()):
                sensor = self.sensors[sensor_id]

                sensor.parse(data)
                sensor.current_packet = packet
        else:
            print(("Invalid packet: " + repr(packet)))


class CommandQueue(object):
    """
    creates a queue for all commands as a nice wrapper for using queues in Python
    """

    def __init__(self):
        self.queue = []

    def put(self, command):
        assert (isinstance(command, Command))
        self.queue.append(command.get_packet())

    def get(self):
        return self.queue.pop(0)

    def is_empty(self):
        return len(self.queue) == 0


def try_sensor(properties):
    global sensor_pool
    sensor_id = max(sensor_pool.sensors.keys()) + 1
    exp_sensor = Sensor(sensor_id, properties)

    packet = ""
    for _ in properties:
        data_type = random.choice(['i', 'u', 'f', 'b'])
        if data_type == 'u' or data_type == 'i':
            int_size = random.choice([8, 16, 32, 64])
            data = random.randint(0, 2 << (int_size - 1) - 1)
            print("%i, %x" % (data, data))
            packet += "%s%x\t" % (data_type, data)
        elif data_type == 'f':
            data = random.random() * 10000
            packet += "%s%0.8x\t" % (data_type,
                                  struct.unpack('<I', struct.pack('<f', data))[0])
        elif data_type == 'b':
            data = random.choice([True, False])
            if data == True:
                data = random.randint(1, 15)
            else:
                data = 0
            packet += "%s%x\t" % (data_type, data)

    exp_sensor.parse(packet[:-1])

    return exp_sensor._properties


def try_command(command_id, data_range, data=None):
    exp_command = Command(command_id, 'test', data_range)
    if data == None:
        data = random.randint(exp_command.range[0], exp_command.range[1])
        print("data:", data)

    exp_command.property_set('test', data)
    return exp_command.get_packet()


class SerialObject(object):
    def __init__(self, object_id, properties):
        """
        formats is the list of formats used by the serial object,
        for this code, the sensors are the serial objects

        :param object_id:
        :param properties:
        :return:
        """

        assert (type(object_id) == int)
        self._properties = self.init_properties(list(properties))

        self.object_id = object_id

        self.current_packet = ""

    @staticmethod
    def init_properties(properties):
        dict_props = OrderedDict()
        for props in properties:
            dict_props[props] = 0

        return dict_props

    def __str__(self):
        """
        reformats the data as a string
        :return:
        """

        return "%s" % (str(self._properties))

    def __repr__(self):
        str_formats = str(self._properties)[1:-1]
        return "%s(%s, %s)" % (self.__class__.__name__, self.object_id,
                               str_formats)


class Sensor(SerialObject):
    def __init__(self, sensor_id, properties):
        super().__init__(sensor_id, properties)

        sensor_pool.add_sensor(self)

    def __getitem__(self, item):
        return self._properties[item]

    @staticmethod
    def format_hex(hex_string, data_format):
        """
        formats each hex string according to what data format was given
        (see SerialObject class for list of formats)

        :param hex_string:
        :param data_format:
        :return:
        """
        if data_format == 'b':
            return bool(int(hex_string, 16))

        elif data_format == 'u':
            return int(hex_string, 16)

        elif data_format == 'i':
            bin_length = len(hex_string) * 4
            raw_int = int(hex_string, 16)
            if (raw_int >> (bin_length - 1)) == 1:
                raw_int -= 2 << (bin_length - 1)
            return int(raw_int)

        elif data_format == 'f':
            if len(hex_string) == 8:
                return struct.unpack('!f', bytes.fromhex(hex_string))[0]
            else:
                return None

        elif data_format == 'd':
            if len(hex_string) == 8:
                return struct.unpack('!d', bytes.fromhex(hex_string))[0]
            else:
                return None

        else:
            # raise ValueError("Invalid data type: %s", str(data_format))
            return None

    def parse(self, hex_string):
        """
        a function to parse data, formatted into hex strings

        Assuming the sensor id and carriage return has been removed from the
        packet since SensorPool has already sorted the data to the appropriate
        sensor.

        :param hex_string:
        :return:
        """

        raw_data = hex_string.split("\t")
        for index, key in enumerate(self._properties.keys()):
            if index < len(raw_data):
                data_type, raw_datum = raw_data[index][0], raw_data[index][1:]
                new_datum = self.format_hex(raw_datum, data_type)
                if new_datum is not None:
                    self._properties[key] = new_datum

    def __str__(self):
        to_string = "["
        for key, value in self._properties.items():
            to_string += "(%s: %s),\t" % (key, value)

        return to_string[:-2] + "]"


class Command(SerialObject):
    def __init__(self, command_id, command_name, data_range, bound=True):
        self.range = data_range
        self.data_type, self.data_len = self.get_type_size(data_range)
        self.command_name = command_name
        super().__init__(command_id, [command_name])

        self.packet_info = "%s\t%s\t" % (self.to_hex(self.object_id, 2),
                                       self.to_hex(self.data_len, 2))
        self.bound = bound

    def __getitem__(self, item):
        return self._properties[item]

    def __setitem__(self, key, value):
        global communicator
        self.property_set(key, value)
        communicator.put(self.get_packet())

    def property_set(self, key, value):
        if self.bound:  # bound by self.data_range
            if value > self.range[1]:
                value = self.range[1]
            elif value < self.range[0]:
                value = self.range[0]
        else:  # acts like mod. Wraps out of bound value to self.data_range
            while value > self.range[1]:
                value -= self.range[1] - self.range[0]
            while value < self.range[0]:
                value += self.range[1] - self.range[0]

        self._properties[key] = value

    @staticmethod
    def get_data_size(data_range):
        length = abs(data_range[1] - data_range[0])

        if length == 0:
            return 8
        else:
            int_length = int(math.log(length, 16)) + 1
            return int_length

    @staticmethod
    def get_type_size(data_range):
        assert len(data_range) == 2
        assert type(data_range[0]) == type(data_range[1])
        if data_range[0] > data_range[1]:
            data_range = (data_range[1], data_range[0])

        if type(data_range[0]) == int:
            if data_range[0] < 0 or data_range[1] < 0:
                data_type = 'i'
            else:
                data_type = 'u'
            data_len = Command.get_data_size(data_range)
        elif type(data_range[0]) == bool:
            data_type = 'b'
            data_len = Command.get_data_size(data_range)
        elif type(data_range[0]) == float:
            data_type = 'f'
            data_len = 64
        else:
            raise ValueError("Range invalid. Invalid data type: %s",
                             str(data_range))
        return data_type, data_len

    @staticmethod
    def to_hex(decimal, length):
        hex_format = "0.%sx" % length
        return ("%" + hex_format) % decimal

    def format_data(self, data):
        """
        formats data for sending over serial

        :param data:
        :param data_format:
        :return:
        """

        if self.data_type == 'b':
            return str(int(bool(data)))

        elif self.data_type == 'u':
            data %= MAXINT
            return self.to_hex(data, self.data_len)

        elif self.data_type == 'i':
            if data < 0:
                data += (2 << (self.data_len * 4 - 1))
            data %= MAXINT
            return self.to_hex(data, self.data_len)

        elif self.data_type == 'f':
            return "%0.8x" % struct.unpack('<I', struct.pack('<f', data))[0]

        elif self.data_type == 'd':
            return "%0.16x" % struct.unpack('<Q', struct.pack('<d', data))[0]

    def get_packet(self):
        """
        creates the packet to send

        :return:
        """

        self.current_packet = self.packet_info + \
            self.format_data(self._properties[self.command_name]) + "\r"

        return self.current_packet


# init sensor pool
sensor_pool = SensorPool()
communicator = None

# --------------------------------------------------
#                    Test cases
# --------------------------------------------------

if __name__ == '__main__':
    def almost_equal(value1, value2, epsilon=0.0005):
        """
        almost_equal is a function created to compare floats to assert the code
        is correct.

        :param value1:
        :param value2:
        :param epsilon:
        :return:
        """
        if type(value1) == list and type(value2) == list:
            # assert len(value1) == len(value2)
            for index in range(len(value1)):
                if type(value1[index]) == float or type(value2[index]) == float:
                    if abs(value1[index] - value2[index]) > epsilon:
                        return False
                else:
                    if value1[index] != value2[index]:
                        return False
            return True
        else:
            return abs(value1 - value2) <= epsilon


    imu = Sensor(0, ['accel_x', 'accel_y', 'accel_z',
                     'gyro_x', 'gyro_y', 'gyro_z',
                     'mag_x', 'mag_y', 'mag_z'])
    imu.parse("u73e1\tu2305\tu5243\t"
              "uaa49\tua4d2\tue674\t"
              "u9627\tud3b2\tu3752")
    assert imu["accel_x"] == 0x73e1
    assert imu["accel_y"] == 0x2305
    assert imu["accel_z"] == 0x5243
    assert imu["gyro_x"] == 0xaa49
    assert imu["gyro_y"] == 0xa4d2
    assert imu["gyro_z"] == 0xe674
    assert imu["mag_x"] == 0x9627
    assert imu["mag_y"] == 0xd3b2
    assert imu["mag_z"] == 0x3752

    imu.parse("i933f\ticae9\ti047f\t"
              "i00d1\tiff35\ti107c\t"
              "i36ed\tic14b\tic349")

    assert imu["accel_x"] == -0x6cc1
    assert imu["accel_y"] == -0x3517
    assert imu["accel_z"] == 0x47f
    assert imu["gyro_x"] == 0xd1
    assert imu["gyro_y"] == -0xcb
    assert imu["gyro_z"] == 0x107c
    assert imu["mag_x"] == 0x36ed
    assert imu["mag_y"] == -0x3eb5
    assert imu["mag_z"] == -0x3cb7

    gps = Sensor(1, ['lat', 'long', 'speed', 'heading', 'hdop'])
    gps.parse("f457ec735\t"
              "f457ed196\t"
              "f457a1718\t"
              "i1a\t"
              "i69")


    assert almost_equal(gps["lat"], 4076.4504098326465)
    assert almost_equal(gps["long"], 4077.0990541993133)
    assert almost_equal(gps["speed"], 4001.4434308771893)
    assert gps["heading"] == 26
    assert gps["hdop"] == 105

    test = Sensor(2, ['value'])
    for value in range(-128, 128):
        if value < 0:
            value += (2 << 7)

        test.parse("i" + Command.to_hex(value, 8))
        assert test["value"] == value

    test = Sensor(3, ['value'])
    for value in range(0, 256):
        test.parse("u" + Command.to_hex(value, 8))
        assert test["value"] == value

    print("Sensor IDs 0...3 have been taken")

else:
    from board.comm import Communicator

    def start(baud=115200, use_handshake=True):
        global communicator
        communicator = Communicator(baud, sensor_pool, use_handshake)
        communicator.start()

    def stop():
        global communicator
        communicator.stop()
        time.sleep(0.005)

    _initial_time = time.time()
    def is_running(threshold=2):
        global communicator
        return (round(time.time() - _initial_time) - communicator.thread_time) <= threshold
