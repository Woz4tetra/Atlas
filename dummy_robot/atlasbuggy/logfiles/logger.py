import os
import struct
import gzip

from atlasbuggy.logfiles import *
from atlasbuggy import project


class Logger:
    """A class for recording data from a robot to a logs file"""

    def __init__(self, file_name, directory):
        # This lock prevents multiple threads writing data at the same time
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
        # self.index = 0

    def open(self):
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        print("Writing to:", self.file_path)

        self.data_file = open(self.file_path, "w+")

        self.is_open = True

    @staticmethod
    def float_to_hex(number):
        return "%0.8x" % struct.unpack('<I', struct.pack('<f', number))[0]

    def record(self, timestamp, whoiam, packet):
        """
        Record incoming packet using the appropriate separator characters
        """
        if self.is_open:
            hex_timestamp = self.float_to_hex(timestamp)

            self.data.append("%s%s%s%s%s\n" % (hex_timestamp, time_whoiam_sep, whoiam, whoiam_packet_sep, packet))

            if len(self.data) > 0xff:
                self.dump_all()

    def dump_all(self):
        while len(self.data) > 0:
            self.data_file.write(self.data.pop(0))

    def compress(self):
        with open(self.file_path, "rb") as file:
            raw_data = file.read()

        compressed_data = gzip.compress(raw_data)
        with open(self.file_path, "wb") as file:
            file.write(compressed_data)


    def close(self):
        if self.is_open:
            self.dump_all()
            self.data_file.close()
            self.compress()
            self.is_open = False
