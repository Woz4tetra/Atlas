import array
import struct


class CommandPool(object):
    def __init__(self, commands):
        """
        Constructor for the CommandPool class. This class continuously listens to what comes in over serial and updates
        the appropriate commands.
        :param commands:
        """
        self.commands = {}

        for command in commands:
            if command.object_id in list(self.commands.keys()):
                raise Exception(
                    "Command ID already taken: " + str(command.object_id))
            else:
                self.commands[command.object_id] = command

    @staticmethod
    def is_hex(character):
        return (ord('0') <= ord(character) <= ord('9') or
                ord('a') <= ord(character) <= ord('f') or
                ord('A') <= ord(character) <= ord('F'))

    @staticmethod
    def is_packet(packet):
        """
        If the packet contains valid characters and is a valid length, it is deemed a valid packet
        :param packet: a packet string
        :return: True or False
        """
        if len(packet) < 2: return False
        # for index in [0, 1, 3, 4] + list(range(6, len(packet) - 1)):
        for index in range(len(packet)):
            if packet[index] != '\t' and not CommandPool.is_hex(packet[index]):
                return False
        return True

    def update(self, packet):
        """
        Update the appropriate command based on the given packet. Call the command's callback function
        Assumes the new line has been stripped.
        :param packet: a packet string
        :return: None
        """
        if self.is_packet(packet):
            command_id = int(packet[0:2], 16)
            if command_id not in self.commands:
                print("Command ID not found: %i. Did you forget to add it to "
                      "comm object?" % command_id)
                return
            if len(packet) == 2:  # use previously sent command
                data = self.commands[command_id].data
            else:
                command_id = int(packet[0:2], 16)
                hex_data = packet[3:]

                if command_id in self.commands.keys():
                    self.commands[command_id].current_packet = packet
                    data = self.commands[command_id].format_data(hex_data)
                else:
                    return  # invalid packet. Do nothing
            self.commands[command_id].callback(data)
            self.commands[command_id].new_data = True


class SerialObject(object):
    def __init__(self, object_id):
        """
        The super class of Command and Sensor.
        :param object_id: A unique identifier of the object (an integer >= 0)
        """
        self.object_id = object_id
        self.current_packet = ""
        self.data = []

    def reset(self):
        """
        Resets the object to initial conditions (called when the main computer restarts communications)
        :return: None
        """
        pass

    def recved_data(self):
        """
        Flags True if new data is received
        :return:
        """
        pass

    def __repr__(self):
        return "%s(%i, %s)" % (
            self.__class__.__name__, self.object_id, self.data)

    def __str__(self):
        return str(self.data)


class Sensor(SerialObject):
    def __init__(self, sensor_id, formats):
        """
        Parses the given formats and initializes internal data
        :param sensor_id: Unique sensor identifier
        :param formats: a string or list of strings.
            i## - signed int. Write i64 to indicate a signed integer with 64 bits
            u## - unsigned int
            f - float (32 bit floating point)
            b - bool
        """
        super().__init__(sensor_id)

        if isinstance(formats, list) or isinstance(formats, tuple):
            self.formats = formats
        else:
            self.formats = [formats]
        self.format_len = []
        for data_format in self.formats:
            if data_format[0] == 'u' or data_format[0] == 'i':
                self.data.append(0)
                self.format_len.append(int(data_format[1:]) // 4)
            elif data_format[0] == 'f':
                self.data.append(0.0)
                self.format_len.append(8)
            elif data_format[0] == 'b':
                self.data.append(False)
                self.format_len.append(1)
            else:
                raise ValueError("Invalid format: %s", str(data_format))

    def to_hex(self, data, length=0):
        """
        Converts an integer to a hex string. length is for leading zeros
        :param data: an integer
        :param length: number of hexadecimal digits the integer has
        :return: the converted integer
        """
        if type(data) == int:
            if data < 0 and length > 0:
                data += (2 << (length * 4 - 1))
            # data %= MAXINT

            hex_format = "0.%sx" % length
            return ("%" + hex_format) % data
        else:
            raise Exception("Data not int type")

    @staticmethod
    def float_to_hex(data):
        return "%0.8x" % (struct.unpack('<I', bytes(array.array('f', [data]))))

    def format_data(self):
        """
        Formats the sensor's data according to the provided formats
        :return:
        """
        hex_string = ""
        # length of data should equal number of formats
        for index in range(len(self.formats)):
            hex_string += self.formats[index][0]
            if self.formats[index] == 'f':
                hex_string += Sensor.float_to_hex(self.data[index])
            elif self.formats[index] == 'b':
                hex_string += "1" if bool(self.data[index]) else "0"
            else:
                hex_string += self.to_hex(self.data[index], self.format_len[index])
            hex_string += '\t'

        return hex_string[:-1]

    def update_data(self):
        """
        When the sensor updates, return the data to be sent over serial
        :return: Depending on the formats provided, return a single value or a list (or tuple) of values
        """
        pass

    def get_packet(self):
        """
        Get the hex string to be sent over serial. Format the data appropriately
        :return:
        """
        self.data = self.update_data()
        try:
            _ = (e for e in self.data)
        except TypeError:
            # print(self.data, 'is not iterable')
            self.data = [self.data]
        self.current_packet = "%s\t%s\r\n" % (self.to_hex(self.object_id, 2), self.format_data())
        return self.current_packet


class Command(SerialObject):
    def __init__(self, command_id):
        super().__init__(command_id)

    def format_data(self, hex_string):
        parsed_data = []
        for value in hex_string.split("\t"):
            data_type = value[0]
            data = value[1:]
            parsed_data.append(self.to_hex(data, data_type))
        if len(parsed_data) == 1:
            parsed_data = parsed_data[0]
        self.data = parsed_data
        return parsed_data

    def to_hex(self, hex_string, data_format):
        if data_format == 'b':
            data = bool(int(hex_string, 16))

        elif data_format == 'i':
            data = int(hex_string, 16)

        elif data_format[0] == 'f':
            # assure length of 8
            input_str = "0" * (8 - len(hex_string)) + hex_string

            data = struct.unpack('!f', bytes.fromhex(input_str))[0]
        else:
            raise ValueError("Invalid data type '%s' for command %s" % (
                data_format[0], repr(self)))

        return data

    def callback(self, data):
        pass


class PollSensor(SerialObject):
    pass
    # TODO: implement poll based sensors. It has a command and sensor. The
    # command is used to tell the sensor to return data
