"""
Contains the Logger class. This class writes incoming packets and their corresponding
whoiam ID and timestamp to a text file. When the program ends, this text file is compressed into a gzip file.
"""

import time
from atlasbuggy.files.atlasbuggyfile import AtlasReadFile, AtlasWriteFile

# logs file data separators (end markers)
time_whoiam_sep = ":"  # timestamp
whoiam_packet_sep = ";"  # data name

# all data before this date may not work correctly with the current code
log_file_type = "gzip"
log_dir = "logs"

default_log_file_name = "%H;%M;%S"
default_log_dir_name = "%Y_%b_%d"

no_timestamp = "-"

packet_types = {
    "object"         : "<",  # from a robot object
    "user"           : "|",  # user logged
    "command"        : ">",  # command sent

    "error"          : "!",  # printed error message
    "error continued": "[",  # error message continued on next line
    "error end"      : "]",  # error message ends

    "debug"          : "?",  # printed debug message
    "debug continued": "(",  # debug message continued on next line
    "debug end"      : ")",  # debug message ends
}


def make_reversible(d):
    d.update({v: k for k, v in d.items()})


make_reversible(packet_types)


class Logger(AtlasWriteFile):
    """A class for recording data from a robot to a logs file"""

    def __init__(self, file_name=None, directory=None):
        if file_name is None:
            file_name = time.strftime(default_log_file_name)
        if directory is None:
            directory = time.strftime(default_log_dir_name)
        elif type(directory) == tuple and None in directory:
            directory = list(directory)
            none_index = directory.index(None)
            directory[none_index] = time.strftime(default_log_dir_name)

            for index, sub_dir in enumerate(directory):
                directory[index] = sub_dir.strip("/")
            directory = "/".join(directory).strip("/")

        super(Logger, self).__init__(file_name, directory, True, log_file_type, log_dir)

        self.line_code = (("%s" * 6) + "\n")

    def record(self, timestamp, whoiam, packet, packet_type):
        """
        Record incoming packet.

        :param timestamp: time packet arrived. -1 if it's an initialization packet
        :param whoiam: whoiam ID of packet (see object.py for details)
        :param packet: packet received by robot port
        :param packet_type: packet decorator. Determines how the packet was used
        :return: None
        """

        if self.is_open():
            if timestamp is None:
                timestamp = no_timestamp
            else:
                assert type(timestamp) == float

            if packet_type == "error" or packet_type == "debug":
                if "\n" in packet:
                    packet = packet.replace("\n", "\n" + packet_types[packet_type + " continued"])
                    packet += "\n" + packet_types[packet_type + " end"]
                    print(repr(packet))

            self.write(self.line_code % (
                packet_types[packet_type], timestamp, time_whoiam_sep, whoiam, whoiam_packet_sep, packet))


class Parser(AtlasReadFile):
    """
    A class for parsing logs files and returning their data nicely.

    This class is meant to be used as an iterator.

    for example:
    parser = Parser("file name", "directory in logs")
    for index, timestamp, whoiam, packet in parser:
        pass

    Parser returns data in the order that it was recorded. If, say, an IMU
    sensor recorded three times and then a GPS recorded once, the parser would
    return the IMU's data three times and then the GPS last
    """

    def __init__(self, file_name, directory=None, start_index=0, end_index=-1):
        """
        :param file_name:
        :param directory:
        :param start_index:
        :param end_index:
        """
        super(Parser, self).__init__(file_name, directory, True, log_file_type, log_dir)

        self.contents = self.contents.split("\n")

        # index variables
        self.start_index = start_index
        self.end_index = end_index
        self.index = self.start_index  # current packet number (or line number)
        self.timestamp = 0

        if self.end_index == -1:
            self.end_index = len(self.contents)

    def __iter__(self):
        """
        Iterate through the file. The recommended use of this class.

        for example:
        parser = Parser("file name", "directory in logs")
        for index, timestamp, name, values in parser:
            pass
        """
        return self

    def next(self):
        """
        While self.content_index hasn't reached the end of the file, parse the current line
        and return the contents. If the line wasn't parsed correctly, StopIteration is raised.
        :return: tuple: (index # (int), timestamp (float), whoiam (string), packet (string))
        """
        if self.index < self.end_index:
            line = self.parse_line()
            self.index += 1
            if line is not None:
                packet_type, timestamp, whoiam, packet = line
                return self.index - 1, packet_type, timestamp, whoiam, packet
        return None

    def parse_line(self):
        """
        Parse the current line using self.content_index and separator globals (e.g. time_whoiam_sep).
        Return the contents found
        :return: timestamp, whoiam, packet; None if the line was parsed incorrectly
        """
        line = self.contents[self.index]
        if len(line) == 0:
            print("Empty line (line #%s)" % self.index)
            return None
        if line[0] not in packet_types.keys():
            print("Invalid packet type: '%s' in line #%s: %s" % (line[0], self.index, line))
            return None
        packet_type = packet_types[line[0]]

        whoiam = ""
        packet = ""

        # the values are from the end of the name to the end of the line
        if len(packet_type) >= len("error") and packet_type[:len("error")] == "error":
            if len(packet_type) == len("error"):
                self.timestamp, time_index, whoiam, whoiam_index = self.find_packet_header(line)
                if self.timestamp == no_timestamp:
                    self.timestamp = 0.0
                packet = "\n----- Error message in log (time: %0.4fs, type: %s) -----\n" % (
                    self.timestamp, whoiam)
                packet += "Traceback (most recent call last):\n"
                packet += line[whoiam_index + len(time_whoiam_sep):]

            elif packet_type[len("error") + 1:] == "continued":
                packet = line[1:]

            elif packet_type[len("error") + 1:] == "end":
                packet = "----- End error message -----\n"

            packet_type = "error"

        elif len(packet_type) >= len("debug") and packet_type[:len("debug")] == "debug":
            if len(packet_type) == len("debug"):
                self.timestamp, time_index, whoiam, whoiam_index = self.find_packet_header(line)
                # if self.timestamp == no_timestamp:
                #     packet = "[debug at initialization from %s]: " % whoiam
                # else:
                #     packet = "[debug at %0.4fs from %s]: " % (self.timestamp, whoiam)
                packet += line[whoiam_index + len(time_whoiam_sep):]

            elif packet_type[len("debug") + 1:] == "continued":
                packet = "\n" + line[1:]

            elif packet_type[len("debug") + 1:] == "end":
                packet = "[end debug from %s]: " % line[1:]

            packet_type = "debug"

        else:
            self.timestamp, time_index, whoiam, whoiam_index = self.find_packet_header(line)
            if self.timestamp == -1:
                print("Invalid timestamp in line #%s: %s" % (self.index, line))
                return None
            if len(whoiam) == 0:
                print("Invalid whoiam in line #%s: %s" % (self.index, line))
                return None
            else:
                packet = line[whoiam_index + len(time_whoiam_sep):]

        return packet_type, self.timestamp, whoiam, packet

    def find_packet_header(self, line):
        # search for the timestamp from the current index to the end of the line
        time_index = line.find(time_whoiam_sep)

        # search for the name from the current index to the end of the line
        whoiam_index = line.find(whoiam_packet_sep)

        if time_index != -1:
            timestamp = line[1:time_index]
            if timestamp != no_timestamp:
                try:
                    timestamp = float(timestamp)
                except ValueError:
                    print("Invalid timestamp:", timestamp)
            else:
                timestamp = no_timestamp
        else:
            timestamp = -1

        if whoiam_index != -1:
            # the name is from the end of the timestamp to name_index
            whoiam = line[time_index + len(time_whoiam_sep): whoiam_index]
        else:
            whoiam = ""

        return timestamp, time_index, whoiam, whoiam_index
