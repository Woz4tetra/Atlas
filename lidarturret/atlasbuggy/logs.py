import time
from datetime import datetime
import pickle
import os
import sys
from threading import Lock

from atlasbuggy import project

# log file data separators (end markers)
time_whoiam_sep = ":\t"  # timestamp
whoiam_packet_sep = ";\t"  # data name

# all data before this date may not work correctly with the current code
obsolete_data = "Nov 14 2016"
log_file_type = "txt"  # easily change file types (not that you should need to)
pickle_file_type = "pkl"

log_directory = ":logs"
pickle_directory = ":pickled"

log_folder_format = '%b %d %Y'
log_file_format = '%H;%M;%S, %a %b %d %Y'


def todays_log_folder():
    """Generate a log folder name based on the current date"""
    return time.strftime(log_folder_format) + "/"


def filename_now():
    return time.strftime(log_file_format)


class Logger:
    """A class for recording data from a robot to a log file"""

    def __init__(self, file_name, directory):
        # Logger can be accessed by the main thread and any robot port threads.
        # This lock prevents multiple threads writing data at the same time
        self.log_lock = Lock()

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

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", os.path.join(directory, file_name))

        self.data_file = open(os.path.join(directory, file_name), 'w+')

        self.time0 = 0
        self.log_started = False

        self.is_open = True

    def start_time(self):
        self.log_lock.acquire()

        self.time0 = time.time()
        self.log_started = True

        self.log_lock.release()

    def record(self, who_i_am, packet):
        """
        Record incoming packet using the appropriate separator characters
        """
        if self.is_open:
            self.log_lock.acquire()

            if not self.log_started:
                timestamp = -1
            else:
                timestamp = time.time() - self.time0
            self.data_file.write(
                str(timestamp) + time_whoiam_sep + who_i_am + whoiam_packet_sep + packet + "\n"
            )

            self.log_lock.release()

    def close(self):
        self.log_lock.acquire()
        self.data_file.close()
        self.log_lock.release()

        self.is_open = False


class Parser:
    """
    A class for parsing log files and returning their data nicely.

    This class is meant to be used as an iterator.

    for example:
    parser = Parser("file name", "directory in logs")
    for index, timestamp, name, values in parser:
        pass

    Parser returns data in the order that it was recorded. If, say, an IMU
    sensor recorded three times and then a GPS recorded once, the parser would
    return the IMU's data three times and then the GPS last
    """

    def __init__(self, file_name, directory=None):
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

        print("Using file named '%s'" % self.file_name)

        # read the whole file as a string
        with open(os.path.join(self.directory, self.file_name), 'r') as data_file:
            self.contents = data_file.read()

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

        self.data = []  # the parsed data of the file
        self.iter_index = 0  # the character index of the file

        pickle_file_name = self.file_name[
                           :-len(log_file_type)] + pickle_file_type

        if self.directory != project.interpret_dir(log_directory):
            sub_directory = self.local_dir
        else:
            sub_directory = ""
        log_pickle_dir = os.path.join(project.interpret_dir(pickle_directory), sub_directory)

        if os.path.isfile(os.path.join(log_pickle_dir, pickle_file_name)):
            print("Using pickled data")
            time0 = time.time()
            with open(os.path.join(log_pickle_dir, pickle_file_name), 'rb') as pickle_file:
                self.data = pickle.load(pickle_file)
            print("Took %s seconds" % (time.time() - time0))
        else:
            # parse the file and write it to a pickle file
            print("Using raw file")
            time0 = time.time()
            self.create_data()
            print("Took %s seconds" % (time.time() - time0))

            if not os.path.isdir(log_pickle_dir):
                os.mkdir(log_pickle_dir)
            with open(os.path.join(log_pickle_dir, pickle_file_name), 'wb') as pickle_file:
                pickle.dump(self.data, pickle_file, pickle.HIGHEST_PROTOCOL)
            print("Wrote pickle to: " + os.path.join(log_pickle_dir, pickle_file_name))

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
        if self.iter_index < len(self.data):
            timestamp, name, values = self.data[self.iter_index]
            self.iter_index += 1
            return self.iter_index - 1, timestamp, name, values
        else:
            raise StopIteration

    def __getitem__(self, item):
        return self.data[item]

    def __len__(self):
        return len(self.data)

    def get(self, instance_num, robot_object_name):
        """Get the nth occurrence of a particular type of data"""
        counter = 0
        # iterate until the nth instance is found
        for index, (timestamp, who_i_am, packet) in enumerate(self.data):
            if who_i_am == robot_object_name:
                if counter == instance_num:
                    return timestamp, index, packet
                counter += 1
        raise ValueError(
            "Sensor '%s' not found in log file..." % robot_object_name)

    def create_data(self):
        """Using the separator flags, parse the file and put it in self.data"""
        index = 0
        while index < len(self.contents):
            end_index = self.contents.find("\n", index)

            # search for the timestamp from the current index to the end of the line
            time_index = self.contents.find(time_whoiam_sep, index, end_index)

            # search for the name from the current index to the end of the line
            whoiam_index = self.contents.find(whoiam_packet_sep, index,
                                              end_index)

            # if the line was parsed correctly
            if time_index != -1 and whoiam_index != -1 and end_index != -1:
                # the timestamp is the beginning of the line to time_index
                timestamp = float(self.contents[index: time_index])

                # the name is from the end of the timestamp to name_index
                who_i_am = self.contents[
                           time_index + len(time_whoiam_sep): whoiam_index]

                # the values are from the end of the name to the end of the line
                packet = self.contents[
                         whoiam_index + len(whoiam_packet_sep):end_index]

                # put it in self.data
                self.data.append((timestamp, who_i_am, packet))

            index = end_index + 1
