"""
Contains the Logger class. This class writes incoming packets and their corresponding
whoiam ID and timestamp to a text file. When the program ends, this text file is compressed into a gzip file.
"""

import os
import struct
import gzip

from atlasbuggy.logfiles import *
from atlasbuggy import project


class Logger:
    """A class for recording data from a robot to a logs file"""

    def __init__(self, file_name, directory):
        # Format the file name. Mac doesn't support colons in file names
        # If file format is provided but not a name, insert timestamp
        if file_name is None or file_name.replace("." + log_file_type, "") == "":
            file_name = filename_now() + "." + log_file_type

        elif len(file_name) < 4 or file_name[-4:] != "." + log_file_type:
            file_name += "." + log_file_type

        # Parse the input directory using the project module
        if directory == ":today":  # for creating logs
            directory = os.path.join(project.interpret_dir(log_directory), todays_log_folder())
        elif directory is None:
            directory = project.interpret_dir(log_directory)
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = os.path.join(project.interpret_dir(log_directory), directory)

        self.file_name = file_name
        self.directory = directory

        self.file_path = os.path.join(self.directory, self.file_name)

        self.data_file = None
        self.is_open = False

        self.data = []

    def open(self):
        """
        Create the directory if it doesn't exist. Open the file for writing

        :return:
        """
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        print("Writing to:", self.file_path)

        self.data_file = open(self.file_path, "w+")

        self.is_open = True

    @staticmethod
    def float_to_hex(number):
        """
        Turn a floating point number to a hexidecimal string according to IEEE 754.
        This meant for timestamps

        :param number: float
        :return: string containing hex representation
        """
        return "%0.8x" % struct.unpack('<I', struct.pack('<f', number))[0]

    def record(self, timestamp, whoiam, packet, packet_type):
        """
        Record incoming packet.

        :param timestamp: time packet arrived. -1 if it's an initialization packet
        :param whoiam: whoiam ID of packet (see robotobject.py for details)
        :param packet: packet received by robot port
        :param packet_type: packet decorator. Determines how the packet was used
        :return: None
        """
        packet_type = packet_types[packet_type]

        if self.is_open:
            # hex_timestamp = self.float_to_hex(timestamp)

            self.data.append("%s%s%s%s%s%s\n" % (
                packet_type, timestamp, time_whoiam_sep, whoiam, whoiam_packet_sep, packet))

            if len(self.data) > 0x1000:
                self.dump_all()

    def dump_all(self):
        """
        Write all data in self.data. This minimizes OS system calls.
        :return: None
        """
        while len(self.data) > 0:
            self.data_file.write(self.data.pop(0))

    def compress(self):
        """
        Compress the written text file into a gzip.
        :return: None
        """
        with open(self.file_path, "rb") as file:
            raw_data = file.read()

        compressed_data = gzip.compress(raw_data)
        with open(self.file_path, "wb") as file:
            file.write(compressed_data)

    def close(self):
        """
        If the file hasn't been closed, close and compress it.
        :return: None
        """
        if self.is_open:
            self.dump_all()
            self.data_file.close()
            self.compress()
            self.is_open = False
