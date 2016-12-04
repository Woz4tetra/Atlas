"""
Handles sensor data sorting, the command queue, and raw data storage.

Classes:
    SensorPool - parses and sorts the incoming packet by giving it to the
        correct sensor
    SerialObject - Internal. The superclass of Sensor and Command.
    Sensor - An object containing data read from the microcontroller with
        the matching sensor ID
    Command - An object that allows data to be sent to the microcontroller

Functions (use with python -i data.py):
    try_sensor - try out a given sensor configuration
    try_command - with a given command, see what is sent over serial
"""

import math
import random
import struct
import time
from collections import OrderedDict
from sys import maxsize as MAXINT


class SensorPool:
    def __init__(self, *sensors):
        self.sensors = {}
        
        self.enable_callbacks = True

        for sensor in sensors:
            self.add_sensor(sensor)

    def add_sensor(self, sensor):
        """
        Adds the sensor to the pool if its sensor ID hasn't been taken already
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
        Takes an incoming packet and, if it's a valid packet, gives it to the
        corresponding sensor for parsing and calls its callback function.
        """

        if self.is_packet(packet):
            sensor_id, data = int(packet[0:2], 16), packet[3:]
            if sensor_id in list(self.sensors.keys()):
                sensor = self.sensors[sensor_id]

                sensor._new_data_received = True
                sensor.sleep_time = time.time() - sensor.prev_time
                sensor.prev_time = time.time()

                sensor.parse(data, packet)

                if sensor.update_fn is not None and self.enable_callbacks:
                    sensor.update_fn()

                return sensor

        return None

    def __len__(self):
        return len(self.sensors)


def try_sensor(*properties, sensor_id=0):
    """
    See what a sensor will return given random data. To test this sensor,
    try (in terminal):
    python3 -i data.py
    try_sensor(["property1", "property2", "property3",])
    """
    exp_sensor = Sensor(sensor_id, 'trial_sensor', properties=properties)

    packet = ""
    for _ in properties:
        data_type = random.choice(['i', 'u', 'f', 'b'])
        if data_type == 'u' or data_type == 'i':
            int_size = random.choice([8, 16, 32, 64])
            data = random.randint(0, 2 << (int_size - 1) - 1)
            packet += "%s%x\t" % (data_type, data)
        elif data_type == 'f':
            data = random.random() * 10000
            packet += "%s%0.8x\t" % (data_type,
                                     struct.unpack('<I',
                                                   struct.pack('<f', data))[0])
        elif data_type == 'b':
            data = random.choice([True, False])
            if data is True:
                data = random.randint(1, 15)
            else:
                data = 0
            packet += "%s%x\t" % (data_type, data)

    data_string = packet[:-1]
    packet = "%0.2x\t%s\r\n" % (sensor_id, data_string)
    exp_sensor.parse(data_string, packet)
    print("packet:", repr(exp_sensor.current_packet))

    return exp_sensor


def try_command(data_range, data=None):
    """
    See what a command packet would look like. To test this command,
    try (in terminal):
    python3 -i data.py
    try_command((-90, 90))
    try_command((False, True))
    import math
    try_command((-math.pi, math.pi))
    """
    if len(Command.used_ids) == 0:
        command_id = 0
    else:
        command_id = Command.used_ids[-1] + 1
    exp_command = Command(command_id, "trial_command", data_range, None)
    if data is None:
        if exp_command.data_type == 'u' or exp_command.data_type == 'i':
            data = random.randint(exp_command.range[0], exp_command.range[1])
        else:
            data = \
                random.random() * (exp_command.range[1] -
                                   exp_command.range[0]) + exp_command.range[0]
        print("data:", data)

    exp_command.value = exp_command.bound_value(data)
    return exp_command.get_packet()


class SerialObject:
    def __init__(self, object_id, name):
        """
        The super class to Sensor and Command. Holds the properties that are
        common between them.
        """

        self.sleep_time = 0.0
        self.prev_time = time.time()

        self.object_id = object_id
        self.current_packet = ""

        self.name = name

        assert (type(object_id) == int)


class Sensor(SerialObject):
    def __init__(self, sensor_id, name, update_fn=None, properties=None):
        """
        Constructor for Sensor. Inherits from SerialObject.
        Initializes the sensor's properties

        Here's an example packet:
        00\tf4609b3c8\tb0\r\n
        00: the sensor ID
        f4609b3c8: the first piece of data. It has the data type float
            4609b3c8 is the hexadecimal representation of 8812.9453125
        b0: the second piece of data. It has the data type bool and has the
            value False

        :param sensor_id: The unique Sensor identifier. Used for matching
            serial packets to Sensors
        :param name: the name of the sensor. Used for logging
        :param properties: A list of strings or a single string
            containing the properties of the object. If None, the only
            property will its own name. When calling get, a property name
            doesn't need to be provided
        :param update_fn: when the sensor gets new data from serial,
            this function is called if not None
        :return: Sensor
        """
        super(Sensor, self).__init__(sensor_id, name)
        self._new_data_received = False

        if properties is None:
            properties = [self.name]
        elif type(properties) == str:
            properties = [properties]
        self._properties = self.init_properties(properties)
        
        self.update_fn = update_fn

    @staticmethod
    def init_properties(properties):
        """Converts a list to an OrderedDict of 0"""
        dict_props = OrderedDict()
        for props in properties:
            dict_props[props] = 0

        return dict_props

    def get(self, item=None, all=False, as_tuple=True):
        # If item is None, it is assumed None was given for properties.
        # This means there is only one property and it should be returned
        if all:
            if as_tuple:
                return tuple(self._properties.values())
            else:
                return self._properties

        if item is None:
            return self._properties[self.name]
        else:
            return self._properties[item]

    def received(self):
        """If new data from serial arrived, this will become True"""
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

    def parse(self, data_string, packet):
        """
        Parses the data portion of a serial packet into meaningful data

        Assuming the sensor id and carriage return has been removed from the
        packet since SensorPool has already sorted the data to the appropriate
        sensor.

        :param data_string: A string of hex characters and data type identifiers
        :param packet: The full packet. This is what current_packet is set to
        :return: None
        """

        raw_data = data_string.split("\t")
        
        # this is why _properties is an ordered dictionary. The properties are
        # matched based on the order they come in
        for index, key in enumerate(self._properties.keys()):
            if index < len(raw_data):
                data_type, raw_datum = raw_data[index][0], raw_data[index][1:]
                new_datum = self.format_hex(raw_datum, data_type)
                if new_datum is not None:
                    self._properties[key] = new_datum
        self.current_packet = packet

    def __repr__(self):
        data = str(self._properties)[12:-1]  # remove OrderedDict from string 
        return "%s(%s, %s)" % (self.__class__.__name__, self.object_id,
                               data)

    def __str__(self):
        to_string = "["
        for key, value in self._properties.items():
            to_string += "(%s: %s),\t" % (key, value)

        return to_string[:-2] + "]"


class Command(SerialObject):
    used_ids = []

    def __init__(self, command_id, name, data_range, communicator,
                 mapping=None, bound_values=True, initial=None):
        """
        Constructor for Command. Inherits from SerialObject.
        Initializes the command's properties

        Here's an example packet if (-90, 90) is given for data range:
        00\t02\tc7\r\n
        00: the command ID
        02: the length of the data (number of characters)
        c7: the data to send. The microcontroller determines how to interpret
            it. Data range is only used to convert it to the correct hex format

        :param command_id: The unique Command identifier. Used for matching
            packets sent to the microcontroller
        :param name: the name of the command. Used for logging
        :param data_range: the possible range of values that can be sent.
            It should be a tuple of size 2. Both values should be of the same
            data type. It doesn't matter what order you put them in. The
            program will swap them to the correct order.
        :param communicator: An instance of the Communicator class. This is so
            Commands can put their data directly to serial
        :param mapping: map specific names to specific values
        :param bound_values: If a value greater than the maximum value
            or less than the minimum value is given, the command will
            cap the value at the maximum or minimum if bound_values is True
            or wrap the value around (similar to modulo) if False
        :param initial: initial value of the command
        :return: Command
        """
        
        # throw an error if a command ID is taken
        if command_id in Command.used_ids:
            raise ValueError("Command ID already in use:", command_id)
        else:
            Command.used_ids.append(command_id)

        super(Command, self).__init__(command_id, name)
        
        # determine the data length, type, and correct range of the command
        self.data_type, self.data_len, self.range = \
            self.get_type_size(data_range)
        
        # to save time, generate the beginning part of the packet
        self.packet_info = "%s\t%s\t" % (self.to_hex(self.object_id, 2),
                                         self.to_hex(self.data_len, 2))

        # this packet tells the microcontroller to use the previous value
        # it received
        self.packet_use_prev = "%s\r\n" % (self.to_hex(self.object_id, 2))

        self.bound = bound_values
        self.communicator = communicator
        
        self.prev_value = None
        
        # set the initial value if None is given
        if initial is None:
            if self.data_type == 'i' or self.data_type == 'u':
                self.value = 0
            elif self.data_type == 'f':
                self.value = 0.0
            elif self.data_type == 'b':
                self.value = False
        
        self.mapping = mapping

    def get(self):
        return self.value

    def send_new_value(self):
        """Send the whole command packet and log it"""
        current_time = time.time()

        if self.communicator.log_data:
            self.communicator.log.enq(self.name, self.value)
        self.communicator.put(self.get_packet())
        time.sleep(0.004)

        self.prev_time = current_time
        self.prev_value = self.value

    def send_prev_value(self):
        """Only send the command ID. Don't log anything"""
        current_time = time.time()
        self.communicator.put(self.packet_use_prev)
        time.sleep(0.004)
        self.prev_time = current_time

    def map_value(self, value):
        """
        Map the value if mapping is not None and if the value is in mapping
        """
        if self.mapping is not None and value in self.mapping:
            return self.mapping[value]
        else:
            return value

    def set(self, value):
        """
        Bound the value and send it over serial. If the value is the same
        as the previous sent value, only send the command ID
        """
        self.value = self.bound_value(self.map_value(value))
        if self.value != self.prev_value:
            self.send_new_value()
        else:
            self.send_prev_value()

    @staticmethod
    def wrap(num, lower, upper):
        """Modulo-like wrapping"""
        while num > upper:
            num -= abs(upper - lower)
        while num < lower:
            num += abs(upper - lower)
        return num

    def bound_value(self, value):
        """Bound the value based on self.bound and self.range"""
        if self.bound:  # bound by self.data_range
            if value > self.range[1]:
                value = self.range[1]
            elif value < self.range[0]:
                value = self.range[0]
        else:  # Wraps out of bound value to self.data_range (modulo)
            value = self.wrap(value, self.range[0], self.range[1])

        return value

    @staticmethod
    def get_data_size(data_range):
        """
        Determines number of hexadecimal characters the data range occupies

        :param data_range: The range of values as supplied by the constructor
        :return: Number of hexadecimal digits of the input
        """
        length = abs(data_range[1] - data_range[0])

        if length == 0:
            # math.log can't handle 0. This occurs if there is only one value
            # the command can send
            return 1
        else:
            # determine number of hexadecimal digits
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
        
        # swap the data range to the correct order
        # 1: larger value
        # 0: smaller value
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
            data_len = 8  # floats occupy 8 hex characters
        else:
            raise ValueError("Range invalid. Invalid data type: %s",
                             str(data_range))
        return data_type, data_len, data_range

    @staticmethod
    def to_hex(decimal, length):
        """
        Convert number to hexadecimal string with a certain number of trailing
        zeros
        """
        hex_format = "0.%sx" % length
        return ("%" + hex_format) % decimal

    def format_data(self, data):
        """
        Converts incoming data to a command packet

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

        # elif self.data_type == 'd':
        #     assert (type(data) == float)
        #     return "%0.16x" % struct.unpack('<Q', struct.pack('<d', data))[0]

    def get_packet(self):
        """
        creates the packet to send
        :return: A string containing a value packet to send over serial
        """

        self.current_packet = self.packet_info + \
                              self.format_data(self.value) + "\r\n"

        return self.current_packet


class CommandArray:
    def __init__(self, command_mapping, name, data_range, communicator,
                 *args, **kwargs):
        """
        Commands on their own don't support sending multiple values. This is
        a wrapper class that supports this.

        :param command_mapping: Map names to command IDs
        :param name: name of the command array
        :param data_range: the data range of the command array. It's assumed
            all commands in this array have the same data range
        :param communicator: An instance of the Communicator class
        :param args: Any extra arguments that Command needs
        :param kwargs: Any extra keyword arguments that Command needs
        """
        self.commands = {}

        # command name to ID number or a list of command IDs
        self.command_mapping = command_mapping
        if type(self.command_mapping) == dict:
            for command_name, command_id in self.command_mapping.items():
                name += " " + command_name
                self.commands[command_name] = \
                    Command(command_id, name, data_range,
                            communicator, *args, **kwargs)
        else:
            for command_id in self.command_mapping:
                name += " " + str(command_id)
                self.commands[command_id] = \
                    Command(command_id, name, data_range,
                            communicator, *args, **kwargs)

    def __getitem__(self, item):
        return self.commands[item]


# class PollSensor(SerialObject):
#     pass
    # TODO: implement poll based sensors. It has a command and sensor.
    # The command is used to tell the sensor to return data
