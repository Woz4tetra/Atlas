import array
import struct


class SensorQueue(object):
    def __init__(self, *sensors):
        self.sensor_index = 0
        self.sensors = {}

        for sensor in sensors:
            if sensor.object_id in list(self.sensors.keys()):
                raise Exception(
                    "Sensor ID already taken: " + str(sensor.object_id))
            else:
                self.sensors[sensor.object_id] = sensor

        self.sensor_ids = sorted(self.sensors.keys())

    def current_index(self):
        sensor_id = self.sensor_ids[self.sensor_index]
        self.sensor_index = (self.sensor_index + 1) % len(self.sensor_ids)
        return sensor_id

    def get(self):
        return self.sensors[self.current_index()].get_packet()


class CommandPool(object):
    def __init__(self, commands):
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
        if len(packet) < 2: return False
        # for index in [0, 1, 3, 4] + list(range(6, len(packet) - 1)):
        for index in range(len(packet)):
            if packet[index] != '\t' and not CommandPool.is_hex(packet[index]):
                return False
        return True

    def update(self, packet):
        if self.is_packet(packet):
            command_id = int(packet[0:2], 16)
            if command_id not in self.commands:
                print("Command ID not found: %i. Did you forget to add it to "
                      "comm object?" % command_id)
                return
            if len(packet) == 2:  # use previously sent command
                data = self.commands[command_id].data[0]
            else:
                data_len = int(packet[3:5], 16)
                hex_data = packet[6: data_len + 6]

                if (len(hex_data) == data_len and
                            command_id in self.commands.keys()):
                    self.commands[command_id].current_packet = packet
                    data = self.commands[command_id].format_data(hex_data)
                else:
                    return  # invalid packet. Do nothing
            self.commands[command_id].callback(data)
            self.commands[command_id].new_data = True


class SerialObject(object):
    def __init__(self, object_id, formats):
        if isinstance(formats, list) or isinstance(formats, tuple):
            self.formats = formats
        else:
            self.formats = [formats]

        self.object_id = object_id

        self.current_packet = ""

        self.data = []
        self.data_len = 0
        self.format_len = []
        for data_format in self.formats:
            if data_format[0] == 'u' or data_format[0] == 'i':
                self.data.append(0)
                self.format_len.append(int(data_format[1:]) // 4)
                self.data_len += int(data_format[1:])
            elif data_format[0] == 'f':
                self.data.append(0.0)
                self.format_len.append(8)
                self.data_len += 8
            elif data_format[0] == 'd':
                self.data.append(0.0)
                self.format_len.append(16)
                self.data_len += 16
            elif data_format[0] == 'b':
                self.data.append(False)
                self.format_len.append(1)
                self.data_len += 1
            else:
                raise ValueError("Invalid format: %s", str(data_format))

    def reset(self):
        pass
        
    def stop(self):
        pass

    def recved_data(self):
        pass

    def __repr__(self):
        return "%s(%i, %s): %s" % (
            self.__class__.__name__, self.object_id, str(self.formats),
            self.data)

    def __str__(self):
        return str(self.data)


class Sensor(SerialObject):
    def __init__(self, sensor_id, formats):
        super().__init__(sensor_id, formats)

    def to_hex(self, data, length=0):
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
        hex_string = ""
        # length of data should equal number of formats
        for index in range(len(self.formats)):
            hex_string += self.formats[index][0]
            if self.formats[index][0] == 'f' or self.formats[index][0] == 'd':
                hex_string += Sensor.float_to_hex(self.data[index])
            elif self.formats[index][0] == 'b':
                hex_string += "1" if bool(self.data[index]) else "0"
            else:
                hex_string += self.to_hex(self.data[index],
                                          self.format_len[index])
            hex_string += '\t'

        return hex_string[:-1]

    def update_data(self):
        pass

    def get_packet(self):
        self.data = self.update_data()
        try:
            _ = (e for e in self.data)
        except TypeError:
            # print(self.data, 'is not iterable')
            self.data = [self.data]
        self.current_packet = "%s\t%s\r\n" % (self.to_hex(self.object_id, 2),
                                              self.format_data())
        return self.current_packet


class Command(SerialObject):
    def __init__(self, command_id, format):
        self.new_data = False
        super().__init__(command_id, [format])

    def format_data(self, hex_string):
        if self.formats[0] == 'b':
            data = bool(int(hex_string, 16))

        elif self.formats[0][0] == 'u':
            data = int(hex_string, 16)

        elif self.formats[0][0] == 'i':
            bin_length = len(hex_string) * 4
            raw_int = int(hex_string, 16)
            if (raw_int >> (bin_length - 1)) == 1:
                raw_int -= 2 << (bin_length - 1)
            data = int(raw_int)

        elif self.formats[0][0] == 'f':
            # assure length of 8
            input_str = "0" * (8 - len(hex_string)) + hex_string

            data = struct.unpack('!f', bytes.fromhex(input_str))[0]
        elif self.formats[0][0] == 'd':
            # assure length of 16
            input_str = "0" * (16 - len(hex_string)) + hex_string

            data = struct.unpack('!d', bytes.fromhex(input_str))[0]
        else:
            raise ValueError("Invalid data type '%s' for command %s" % (
                self.formats[0][0], repr(self)))

        self.data[0] = data
        return data

    def recved_data(self):
        if self.new_data:
            self.new_data = False
            return True
        else:
            return False

    def callback(self, data):
        pass


class PollSensor(SerialObject):
    pass
    # TODO: implement poll based sensors. It has a command and sensor. The
    # command is used to tell the sensor to return data
