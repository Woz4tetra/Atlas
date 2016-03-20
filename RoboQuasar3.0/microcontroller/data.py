"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/6/2016
=========

Handles sensor data sorting, the command queue, and raw data storage.

Classes:
    SensorPool - parses and sorts the incoming packet by giving it to the
        correct sensor
    SerialObject - Internal. The superclass of Sensor and Command.
    Sensor - An object containing data read from the micro-controller with
        the matching sensor ID
    Command - An object that allows data to be sent to the micro-controller

Functions (use with python -i data.py):
    try_sensor - given a list of property names (strings), this function
        allows you to test what the output of your sensor would be
    try_command - this functions returns the packet that
        would be written to serial given typical inputs
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
        """
        Constructor for SensorPool. Initializes sensor pool dictionary as empty

        :return: SensorPool object
        """



        self.sensors = {}

    def add_sensor(self, sensor):
        """
        add sensors to self.sensors with non-repeating sensor IDs

        :param sensor: A sensor object
        :return: None
        """
        if sensor.object_id in list(self.sensors.keys()):
            raise Exception(
                    "Sensor ID already taken: " + str(sensor.object_id))
        elif not isinstance(sensor, Sensor):
            raise TypeError(
                    "Parameter is not of the type sensor: " + repr(sensor))
        else:
            self.sensors[sensor.object_id] = sensor

    def is_packet(self, packet):
        """
        Makes sure the packet is the right format and was safely delivered.
        A packet must be 5 characters or more, contain some or all of the
        following characters:
            0 1 2 3 4 5 6 7 8 9 a b c e d f i u \t
        has a \t character at index 2, and must only have the type identifying
        characters f, i, b, or u at index 3

        If any of these conditions are invalid, the packet is invalid

        :param packet: A string that may or may not be a valid packet
        :return: True or False depending on if the packet is valid
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
        updates the sensors' data when called and, if correct, parses and
        replaces the old sensor data

        :param packet: A string that may or may not be a valid packet
        :return: None
        """

        packet = packet.decode('ascii')
        if self.is_packet(packet):
            sensor_id, data = int(packet[0:2], 16), packet[3:]
            if sensor_id in list(self.sensors.keys()):
                sensor = self.sensors[sensor_id]

                sensor._new_data_received = True
                sensor.sleep_time = time.time() - sensor.prev_time
                sensor.prev_time = time.time()

                sensor.parse(data)
                sensor.current_packet = packet
        else:
            print("Invalid packet: " + repr(packet))


def try_sensor(sensor_id, properties):
    global sensor_pool
    # sensor_id = max(sensor_pool.sensors.keys()) + 1
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
                                     struct.unpack('<I',
                                                   struct.pack('<f', data))[0])
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
        Constructor for SerialObject.
        Takes a list of strings (properties) and creates an internal dictionary
        in which data will be stored.

        :param object_id: The unique SerialObject identifier. Used for matching
            serial packets to Sensors or allowing micro-controllers to identify
            Commands.
        :param properties: A list of strings containing the properties of the
            object
        :return: SerialObject
        """

        self.sleep_time = 0.0
        self.prev_time = time.time()

        assert (type(object_id) == int)
        if type(properties) == str:
            properties = [properties]
        elif type(properties) != list:
            properties = list(properties)
        self._properties = self.init_properties(properties)

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
        """
        Constructor for Sensor. Inherits from SerialObject.
        Adds self to sensor_pool (a module internal object)

        :param sensor_id: The unique Sensor identifier. Used for matching
            serial packets to Sensors or allowing micro-controllers to identify
            Commands.
        :param properties: A list of strings containing the properties of the
            object
        :return: Sensor
        """
        super().__init__(sensor_id, properties)
        self._new_data_received = False

        sensor_pool.add_sensor(self)

    def __getitem__(self, item):
        return self._properties[item]

    def received(self):
        if self._new_data_received:
            self._new_data_received = False
            return True
        else:
            return False

    @staticmethod
    def format_hex(hex_string, data_format):
        """
        Formats each hex string according to what data format was given.
        f = float
        b = bool
        u = unsigned int
        i = int

        :param hex_string: A string of hex characters
        :param data_format: A string containing a data type format
        :return: a float, bool, or int depending on the input data type
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

    def parse(self, data_string):
        """
        a function to parse data, formatted into hex strings

        Assuming the sensor id and carriage return has been removed from the
        packet since SensorPool has already sorted the data to the appropriate
        sensor.

        :param data_string: A string of hex characters and data type identifiers
        :return: None
        """

        raw_data = data_string.split("\t")
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
        """
        Constructor for Command. Inherits from SerialObject.

        :param command_id: The unique Command identifier. Allows
            micro-controllers to identify Commands.
        :param command_name: A string containing the name of property that the
            command modifies
        :param data_range: The range of values that the command can send. This
            determines the data type and size. It doesn't matter if the smaller
            value is first or second.
        :param bound: The value bound type. If True and a value outside
            data_range is given, it will be truncated to that bound, if False
            and a value outside data_range is given, it will wrap around to the
            appropriate value (kind of like overflow or modulo)
        :return: Command object
        """
        self.data_type, self.data_len, self.range = self.get_type_size(
                data_range)
        self.command_name = command_name
        super().__init__(command_id, [command_name])

        self.packet_info = "%s\t%s\t" % (self.to_hex(self.object_id, 2),
                                         self.to_hex(self.data_len, 2))
        self.bound = bound

    def __getitem__(self, item):
        return self._properties[item]

    def __setitem__(self, key, value):
        """
        Allows properties to be set through python's dictionary syntax

        command_example["property"] = value

        :param key: A string with the name of the property
        :param value: A number to send via the command object
        :return: None
        """
        global communicator
        self.property_set(key, value)
        self.sleep_time = time.time() - self.prev_time
        self.prev_time = time.time()
        communicator.put(self.get_packet())

    def property_set(self, key, value):
        if self.bound:  # bound by self.data_range
            if value > self.range[1]:
                value = self.range[1]
            elif value < self.range[0]:
                value = self.range[0]
        else:  # Wraps out of bound value to self.data_range (modulo)
            value %= self.range[1] - self.range[0]

        self._properties[key] = value

    @staticmethod
    def get_data_size(data_range):
        """
        Gets the number of digits of a hexidecimal number

        :param data_range: The range of values as supplied by the constructor
        :return: Number of digits of the input
        """
        length = abs(data_range[1] - data_range[0])

        if length == 0:
            return 8
        else:
            int_length = int(math.log(length, 16)) + 1
            return int_length

    @staticmethod
    def get_type_size(data_range):
        """
        Get the data type of the command based on the command range

        :param data_range: The range of values as supplied by the constructor
        :return: The data type, the data length, and the data range
            (this method swaps the values if the larger value comes first)
        """
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
        return data_type, data_len, data_range

    @staticmethod
    def to_hex(decimal, length):
        hex_format = "0.%sx" % length
        return ("%" + hex_format) % decimal

    def format_data(self, data):
        """
        formats data for sending over serial

        :param data: A number that matches the command's data type (exception:
            you may supply integers for booleans. They will be interpreted
            according to python's bool() function)
        :return: A hex string containing the formatted number
        """

        if self.data_type == 'b':
            assert (type(data) == bool or type(data) == int)
            return str(int(bool(data)))

        elif self.data_type == 'u':
            assert (type(data) == int)
            data %= MAXINT
            return self.to_hex(data, self.data_len)

        elif self.data_type == 'i':
            assert (type(data) == int)
            if data < 0:
                data += (2 << (self.data_len * 4 - 1))
            data %= MAXINT
            return self.to_hex(data, self.data_len)

        elif self.data_type == 'f':
            assert (type(data) == float)
            return "%0.8x" % struct.unpack('<I', struct.pack('<f', data))[0]

        elif self.data_type == 'd':
            assert (type(data) == float)
            return "%0.16x" % struct.unpack('<Q', struct.pack('<d', data))[0]

    def get_packet(self):
        """
        creates the packet to send
        :return: A string containing a value packet to send over serial
        """

        self.current_packet = self.packet_info + \
                              self.format_data(self._properties[
                                                   self.command_name]) + "\r"

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
    from microcontroller.comm import Communicator

    reset_command = Command(255, 'reset', (False, True))

    def reset():
        global reset_command
        communicator.serial_ref.write(struct.pack("B", 4))
        reset_command['reset'] = True

    def start(baud=115200, use_handshake=True, check_status=False):
        global communicator
        communicator = Communicator(baud, sensor_pool, use_handshake)
        communicator.start()
        if check_status:
            status = [False] * 5
            status_index = 0

            print("Checking if board is alive...")
            while not all(status):
                if is_running():
                    status[status_index] = True
                    status_index += 1
                print(".")
                time.sleep(2)
            print("\nIt's alive!")
        reset()

    def stop():
        global communicator
        communicator.stop()
        time.sleep(0.005)


    _initial_time = time.time()


    def is_running(threshold=2):
        global communicator, _initial_time
        status = (round(time.time() - _initial_time) - communicator.thread_time) <= threshold
        if status is True:
            communicator.start_time = _initial_time = time.time()
        return status
