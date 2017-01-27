"""
Contains the Parser class. This class decompresses and parses log files for visualization and post analysis.
"""
import sys
import os
import gzip
import struct
from datetime import datetime

from atlasbuggy.logfiles import *
from atlasbuggy import project


class Parser:
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
        # pick a subdirectory of logs
        self.directory = project.parse_dir(
            directory, log_directory,
            lambda x: datetime.strptime(x, log_folder_format)
        )

        # Get the selected local directory
        self.local_dir = self.directory[self.directory.rfind("/", 0, -1) + 1:]

        # parse the file name. Get the full directory of the file
        self.file_name = project.get_file_name(
            file_name, self.directory, log_file_type)
        self.file_name_no_ext = self.file_name.split(".")[0]

        self.file_path = os.path.join(self.directory, self.file_name)

        print("Using file named '%s'" % self.file_name)

        # decompress the file and put the contents into self.contents
        with open(self.file_path, "rb") as data_file:
            self.contents = gzip.decompress(data_file.read()).decode('utf-8').split("\n")

        # index variables
        self.start_index = start_index
        self.end_index = end_index
        self.index = start_index  # current packet number (or line number)

        if self.end_index == -1:
            self.end_index = len(self.contents)

        # try to parse the name as a timestamp. If it succeeds, see if the
        # file is obsolete. Otherwise, do nothing
        try:
            if (datetime.strptime(self.directory.split("/")[-2], '%b %d %Y') <
                    datetime.strptime(obsolete_data, '%b %d %Y')):
                print("WARNING: You are using a data set that is obsolete "
                      "with the current parser. Continue? (y/n)", end="")
                proceed = None
                while proceed != "" and proceed != "y" and proceed != "n":
                    proceed = input(": ").lower()
                if proceed == "n":
                    sys.exit(1)
        except ValueError:
            pass

    def __len__(self):
        return len(self.contents)

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

    @staticmethod
    def hex_to_float(hex_string):
        """
        Turn a hexidecimal string to a floating point number according to IEEE 754.
        This meant for timestamps

        :param hex_string: string containing hex representation
        :return: floating point equivalent
        """
        return struct.unpack('!f', bytes.fromhex(hex_string))[0]

    def parse_line(self):
        """
        Parse the current line using self.content_index and separator globals (e.g. time_whoiam_sep).
        Return the contents found
        :return: timestamp, who_i_am, packet; None if the line was parsed incorrectly
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
            timestamp = self.hex_to_float(line[1:time_index])

            # the name is from the end of the timestamp to name_index
            who_i_am = line[time_index + len(time_whoiam_sep): whoiam_index]

            # the values are from the end of the name to the end of the line
            packet = line[whoiam_index + len(whoiam_packet_sep):]

            return packet_type, timestamp, who_i_am, packet
        else:
            return None
