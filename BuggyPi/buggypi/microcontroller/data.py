"""
Written by Ben Warwick

data.py, written for BuggyPi
Version 6/6/2016
=========

Handles sensor data sorting, the command queue, and raw data storage.

Classes:
    SensorPool - parses and sorts the incoming packet by giving it to the
        correct sensor
    SerialObject - Internal. The superclass of Sensor and Command.
    Sensor - An object containing data read from the microcontroller with
        the matching sensor ID
    Command - An object that allows data to be sent to the microcontroller

Functions (use with python -i data.py):
    try_sensor - given a list of property names (strings), this function
        allows you to test what the output of your sensor would be
    try_command - this functions returns the packet that
        would be written to serial given typical inputs

Usage
-----

from buggypi.robot import *

class MyRobotRunner:
    def __init__(self):
        sensors = dict(
            encoder=dict(sensor_id=0, properties='counts',
                         update_fn=lambda: self.encoder_updated()),
            gps=dict(sensor_id=1, properties=['long', 'lat', 'fix'],
                     update_fn=lambda: self.gps_updated()),
            imu=dict(sensor_id=2, properties='yaw',
                     update_fn=lambda: self.yaw_updated()),
        )
        commands = dict(
            leds=dict(command_id=0, properties=["red", "yellow", "green", "blue"]),
            servo=dict(command_id=1),
            motors=dict(command_id=2)
        )
        self.robot = Robot(sensors, commands, ...)
"""

import math
import random
import struct
import time
from collections import OrderedDict
from sys import maxsize as MAXINT


class SensorPool:
    def __init__(self, *sensors):
        """
        Sensor pool constructor

        :param sensors: all sensor instances being used. If sensor ID's conflict, an error is thrown
        """
        self.sensors = {}

        for sensor in sensors:
            self.add_sensor(sensor)

    def add_sensor(self, sensor):
        """
        Adds a sensor to the internal dictionary

        :param sensor: An instance of the Sensor class
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
        replaces the old sensor data. If an update function is specified, it is called.

        :param packet: A string that may or may not be a valid packet
        :return: the sensor that was identified. If None, the packet was invalid
        """

        if self.is_packet(packet):
            sensor_id, data = int(packet[0:2], 16), packet[3:]
            if sensor_id in list(self.sensors.keys()):
                sensor = self.sensors[sensor_id]

                sensor._new_data_received = True
                sensor.sleep_time = time.time() - sensor.prev_time
                sensor.prev_time = time.time()

                sensor.parse(data)
                sensor.current_packet = packet

                sensor.update_fn()

                return sensor

        return None

    def __len__(self):
        return len(self.sensors)


class SerialObject:
    def __init__(self, object_id, name, properties):
        """
        Constructor for SerialObject.

        :param object_id: The unique SerialObject identifier. Used for matching
            serial packets to Sensors or allowing micro-controllers to identify
            Commands.
        :param properties: The properties of the object.
            None - assumes the name of the object is the only property.
            a string - the object has one property
            a list of strings - the object has multiple properties
        """

        self.sleep_time = 0.0
        self.prev_time = time.time()

        self.object_id = object_id
        self.current_packet = ""

        self.name = name

        if properties is None:
            properties = [self.name]
        elif type(properties) == str:
            properties = [properties]
        self._properties = self.init_properties(properties)

    def get(self, item=None):
        """
        Get a property of the object. If None is provided, it is assumed that None was provided for properties
        and the sensor's name is used instead (this is for logging purposes

        :param item: The property name to get
        :return: The value of the corresponding property
        """
        if item is None:
            return self._properties[self.name]
        else:
            return self._properties[item]

    @staticmethod
    def init_properties(properties):
        """
        Initialize properties as an ordered dictionary
        :param properties: a list of properties (strings)
        :return: an ordered dictionary of the sensor's properties
        """
        dict_props = OrderedDict()
        for props in properties:
            dict_props[props] = 0

        return dict_props


class Sensor(SerialObject):
    def __init__(self, sensor_id, name, update_fn, properties=None):
        """
        Constructor for Sensor. Inherits from SerialObject.

        :param sensor_id: The unique Sensor identifier. Used for matching
            serial packets to Sensors or allowing micro-controllers to identify
            Commands.
        :param properties: A list of strings containing the properties of the
            object
        """
        super(Sensor, self).__init__(sensor_id, name, properties)

        self._new_data_received = False
        self.update_fn = update_fn

    def update_fn(self):
        """
        This is called every time serial discovers that the sensor has been updated
        :return: None
        """
        pass

    def received(self):
        """
        Flags as True if the sensor received new data

        :return: True or False depending on if the sensor updated since the last call of this function
        """
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

        Assuming the sensor id and new line have been removed from the packet.

        :param data_string: A string of hex characters and data type identifiers
        :return: None, updates the properties of the sensor internally
        """

        raw_data = data_string.split("\t")
        for index, key in enumerate(self._properties.keys()):
            if index < len(raw_data):
                data_type, raw_datum = raw_data[index][0], raw_data[index][1:]
                new_datum = self.format_hex(raw_datum, data_type)
                if new_datum is not None:
                    self._properties[key] = new_datum

    def __repr__(self):
        str_formats = str(self._properties)[1:-1]
        return "%s(%s, %s)" % (self.__class__.__name__, self.object_id,
                               str_formats)

    def __str__(self):
        to_string = "["
        for key, value in self._properties.items():
            to_string += "(%s: %s),\t" % (key, value)

        return to_string[:-2] + "]"


class Command(SerialObject):
    used_ids = []

    def __init__(self, command_id, name, communicator, properties=None):
        """
        Constructor for the Command class.

        :param command_id: A unique value for the command. If there any duplicates, an error is thrown
        :param name: The name of the command
        :param communicator: An instance of the microcontroller.comm.Communicator class
        :param properties: A list of strings, a string, or None signifying the properties of the command.
            If None, the name of the command is used
        """
        if command_id in Command.used_ids:
            raise ValueError("Command ID already in use:", command_id)
        else:
            Command.used_ids.append(command_id)

        super(Command, self).__init__(command_id, name, properties)

        self.packet_info = "%0.2x" % self.object_id
        self.communicator = communicator
        self.prev_properties = OrderedDict()

    def send_new(self):
        """
        Send a full new packet with the updated data
        :return: None
        """
        if self.communicator.log_data:
            self.communicator.log.enq(self.name, self._properties)
        self.communicator.put(self.get_packet())
        time.sleep(0.004)

        self.prev_properties = self._properties.copy()

    def send_prev(self):
        """
        Send just the command ID. This tell the microcontroller to use the previous value
        :return: None
        """
        current_time = time.time()
        self.communicator.put(self.packet_info)  # only send command ID
        time.sleep(0.004)
        self.prev_time = current_time

    def set(self, value=None, **values):
        """
        Set the property or properties of the command and send it to the microcontroller
        :param value: If None was specified for the command, update the single property of the command
        :param values: If a string or list of strings was provided, update those properties
        :return: None
        """
        if value is not None and len(values) == 0:
            if value != self.prev_properties[self.name]:
                self._properties[self.name] = value
                values_changed = True
            else:
                values_changed = False
        else:
            values_changed = False
            for name, value in values.items():
                self._properties[name] = value
                if self._properties[name] != self.prev_properties[name]:
                    values_changed = True
            
        if values_changed:
            self.send_new()
        else:
            self.send_prev()

    def format_data(self):
        """
        formats the command's properties for sending over serial

        :return: A hex string containing the formatted properties
        """
        formatted = ""
        for value in self._properties.values():
            if type(value) == bool:
                formatted += "b%s\t" % str(int(value))
            elif type(value) == int:
                value %= MAXINT
                formatted += "i%x\t" % value
            elif type(value) == float:
                formatted += "f%0.8x\t" % struct.unpack('<I', struct.pack('<f', value))[0]
        return formatted[:-1]  # remove trailing tab

    def get_packet(self):
        """
        creates the packet to send
        :return: A string containing a packet to send over serial
        """
        self.current_packet = "%s\t%s\r\n" % (self.packet_info, self.format_data())

        return self.current_packet


class PollSensor(SerialObject):
    pass
    # TODO: implement poll based sensors. It has a command and sensor. The command is used to tell the sensor to return data
