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
    for index, timestamp, name, values in parser:
        pass

    Parser returns data in the order that it was recorded. If, say, an IMU
    sensor recorded three times and then a GPS recorded once, the parser would
    return the IMU's data three times and then the GPS last
    """

    def __init__(self, file_name, directory=None, start_index=0, end_index=-1):
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

        # read the whole file as a string
        with open(self.file_path, "rb") as data_file:
            self.contents = gzip.decompress(data_file.read()).decode('utf-8')
        self.iter_index = 0
        self.content_index = 0

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
        if self.content_index < len(self.contents):
            line = self.parse_line()
            if line is not None:
                timestamp, name, values = line
                self.iter_index += 1
                return self.iter_index - 1, timestamp, name, values
        raise StopIteration

    @staticmethod
    def hex_to_float(hex_string):
        return struct.unpack('!f', bytes.fromhex(hex_string))[0]

    def parse_line(self):
        end_index = self.contents.find("\n", self.content_index)

        # search for the timestamp from the current index to the end of the line
        time_index = self.contents.find(time_whoiam_sep, self.content_index, end_index)

        # search for the name from the current index to the end of the line
        whoiam_index = self.contents.find(whoiam_packet_sep, self.content_index,
                                          end_index)

        # if the line was parsed correctly
        if time_index != -1 and whoiam_index != -1 and end_index != -1:
            # the timestamp is the beginning of the line to time_index
            timestamp = self.hex_to_float(self.contents[self.content_index: time_index])

            # the name is from the end of the timestamp to name_index
            who_i_am = self.contents[time_index + len(time_whoiam_sep): whoiam_index]

            # the values are from the end of the name to the end of the line
            packet = self.contents[whoiam_index + len(whoiam_packet_sep):end_index]

            self.content_index = end_index + 1

            return timestamp, who_i_am, packet
        else:
            return None
