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

default_log_file_name = "%H;%M.%S"
default_log_dir_name = "%Y_%b_%d"

packet_types = {
    "object" : "<",  # from a robot object
    "user"   : "|",  # user logged
    "command": ">",  # command sent
    "error"  : "!",  # printed error message

    "<"      : "object",
    "|"      : "user",
    ">"      : "command",
    "!"      : "error",
}

no_timestamp = "-"


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
        :param whoiam: whoiam ID of packet (see robotobject.py for details)
        :param packet: packet received by robot port
        :param packet_type: packet decorator. Determines how the packet was used
        :return: None
        """

        if self.is_open():
            if timestamp is None:
                timestamp = no_timestamp

            packet_type = packet_types[packet_type]
            self.write(self.line_code % (
                packet_type, timestamp, time_whoiam_sep, whoiam, whoiam_packet_sep, packet))


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

        # index variables
        self.start_index = start_index
        self.end_index = end_index
        self.index = start_index  # current packet number (or line number)

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

    def __next__(self):
        """
        While self.content_index hasn't reached the end of the file, parse the current line
        and return the contents. If the line wasn't parsed correctly, StopIteration is raised.
        :return: tuple: (index # (int), timestamp (float), whoiam (string), packet (string))
        """
        if self.index < self.end_index:
            line = self.parse_line()
            if line is not None:
                packet_type, timestamp, whoiam, packet = line
                self.index += 1
                return self.index - 1, packet_type, timestamp, whoiam, packet
        raise StopIteration

    def parse_line(self):
        """
        Parse the current line using self.content_index and separator globals (e.g. time_whoiam_sep).
        Return the contents found
        :return: timestamp, whoiam, packet; None if the line was parsed incorrectly
        """
        line = self.contents[self.index]

        # search for the timestamp from the current index to the end of the line
        time_index = line.find(time_whoiam_sep)

        # search for the name from the current index to the end of the line
        whoiam_index = line.find(whoiam_packet_sep)

        # if the line was parsed correctly
        if time_index != -1 and whoiam_index != -1:
            if line[0] not in packet_types.keys():
                return None
            packet_type = packet_types[line[0]]

            # the timestamp is the beginning of the line to time_index
            timestamp = line[1:time_index]
            if timestamp != no_timestamp:
                timestamp = float(timestamp)

            # the name is from the end of the timestamp to name_index
            whoiam = line[time_index + len(time_whoiam_sep): whoiam_index]

            # the values are from the end of the name to the end of the line
            if packet_type == "error":
                packet = "\n\n----- Error message in log (time: %0.4fs, type: %s) -----\n" % (timestamp, whoiam)
                packet += "Traceback (most recent call last):\n"
                packet += line[whoiam_index + len(whoiam_packet_sep):] + "\n"
                for other_lines in self.contents[self.index + 1:]:
                    packet += other_lines + "\n"
                packet += "----- End error message -----\n"
            else:
                packet = line[whoiam_index + len(whoiam_packet_sep):]

            return packet_type, timestamp, whoiam, packet
        else:
            return None
