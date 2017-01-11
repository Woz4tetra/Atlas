import os

from atlasbuggy.logs import *
from atlasbuggy import project


class Logger:
    """A class for recording data from a robot to a log file"""

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

        self.data_file = None
        self.is_open = False

    def open(self):
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        print("Writing to:", os.path.join(self.directory, self.file_name))

        self.data_file = open(os.path.join(self.directory, self.file_name), 'w+')
        self.is_open = True

    def record(self, timestamp, who_i_am, packet):
        """
        Record incoming packet using the appropriate separator characters
        """
        if self.is_open:
            self.data_file.write(
                "%s%s%s%s%s\n" % (timestamp, time_whoiam_sep, who_i_am, whoiam_packet_sep, packet)
            )

    def close(self):
        if self.is_open:
            self.data_file.close()
            self.is_open = False
