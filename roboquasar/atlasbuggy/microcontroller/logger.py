"""
This file handles all interactions with log files and maps. This includes
creating them (done in real time) and parsing them (for playing them back).
"""

import os
import sys
import time
from datetime import datetime
import pickle

from atlasbuggy import project

# log file data separators (end markers)
time_name_sep = ":\t"  # timestamp
name_values_sep = ";\t"  # data name
values_sep = "|\t"  # data value start
datum_sep = ",\t"  # data value separator

# all data before this date may not work correctly with the current code
obsolete_data = "Jun 22 2016"
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
        # Format the file name. Mac doesn't support colons in file names
        # If file format is provided but not a name, insert timestamp
        if file_name is None or \
                        file_name.replace("." + log_file_type, "") == "":
            file_name = filename_now() + "." + log_file_type

        elif len(file_name) < 4 or file_name[-4:] != "." + log_file_type:
            file_name += "." + log_file_type

        # Parse the input directory using the project module
        if directory == ":today":  # for creating logs
            directory = project.interpret_dir(
                log_directory) + todays_log_folder()
        elif directory is None:
            directory = project.interpret_dir(log_directory)
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = project.interpret_dir(log_directory) + directory

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", directory + file_name)

        self.time0 = time.time()
        self.log_start = self.time0

        self.data_file = open(directory + file_name, 'w+')

        self.queue = []

    def enq(self, value_name, data, timestamp=None):
        """Queue data for recording"""
        if timestamp is None:
            timestamp = time.time() - self.time0
        self.queue.append((timestamp, value_name, data))

    def record(self):
        """
        Empty the queue. Put each new piece of data on a new line and format it
        """
        while len(self.queue) > 0:
            timestamp, value_name, data = self.queue.pop(0)
            line = str(timestamp) + time_name_sep + str(
                value_name) + name_values_sep
            if isinstance(data, dict):
                for prop_name, value in data.items():
                    line += prop_name + datum_sep + str(value) + values_sep
                line = line[:-len(values_sep)] + "\n"
            else:
                line += str(data) + "\n"
            self.data_file.write(line)

    def close(self):
        """Empty the queue and close the file"""
        self.record()
        self.data_file.close()


def convert_str(string):
    """Semi-clunky way of parsing strings into the correct data type"""
    if string == "True":
        return True
    if string == "False":
        return False

    trys = [lambda s: int(s), lambda s: float(s), lambda s: str(s)]
    for func in trys:
        try:
            return func(string)
        except ValueError:
            pass
    return None


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

    def __init__(self, file_name, directory=None, start_index=0, end_index=-1):
        # pick a subdirectory of logs
        self.directory = project.parse_dir(directory, log_directory,
                                           lambda x: datetime.strptime(x,
                                                                       log_folder_format))

        # for external use. Get the picked local directory
        self.local_dir = self.directory[self.directory.rfind("/", 0, -1) + 1:]

        # parse the file name. Get the full directory of the file
        self.file_name = project.get_file_name(
            file_name, self.directory, log_file_type)
        self.file_name_no_ext = self.file_name.split(".")[0]

        print("Using file named '%s' in directory '%s'" % (
            self.file_name, self.directory))

        # read the whole file as a string
        with open(self.directory + self.file_name, 'r') as data_file:
            self.contents = data_file.read()

        # try to parse the name as a timestamp. If it succeeds, see if the
        # file is obsolete. Otherwise, do nothing
        try:
            if (datetime.strptime(self.directory.split("/")[-2], '%b %d %Y') <=
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
        log_pickle_dir = project.interpret_dir(
            pickle_directory) + self.local_dir

        if os.path.isfile(log_pickle_dir + pickle_file_name):
            print("Using pickled data")
            time0 = time.time()
            with open(log_pickle_dir + pickle_file_name, 'rb') as pickle_file:
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
            with open(log_pickle_dir + pickle_file_name, 'wb') as pickle_file:
                pickle.dump(self.data, pickle_file, pickle.HIGHEST_PROTOCOL)
            print("Wrote pickle to: " + log_pickle_dir + pickle_file_name)

        self.data = self.data[start_index: end_index]

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

    def get(self, instance_num, sensor_name):
        """Get the nth occurrence of a particular type of data"""
        counter = 0
        # iterate until the nth instance is found
        for index, (timestamp, name, values) in enumerate(self.data):
            if name == sensor_name:
                if counter == instance_num:
                    return timestamp, index, values
                counter += 1
        raise ValueError("Sensor '%s' not found in log file..." % sensor_name)

    def create_data(self):
        """Using the separator flags, parse the file and put it in self.data"""
        index = 0
        while index < len(self.contents):
            end_index = self.contents.find("\n", index)

            # search for the timestamp from the current index to the end of the line
            time_index = self.contents.find(time_name_sep, index, end_index)

            # search for the name from the current index to the end of the line
            name_index = self.contents.find(name_values_sep, index, end_index)

            # if the line was parsed correctly
            if time_index != -1 and name_index != -1 and end_index != -1:
                # the timestamp is the beginning of the line to time_index
                timestamp = float(self.contents[index: time_index])

                # the name is from the end of the timestamp to name_index
                name = self.contents[
                       time_index + len(time_name_sep): name_index]

                # the values are from the end of the name to the end of the line
                value_content = self.contents[
                                name_index + len(name_values_sep):end_index]

                # parse the values and put them in a dictionary
                values = {}
                for data in value_content.split(values_sep):
                    if datum_sep in data:
                        datum_name, datum_value = data.split(datum_sep)
                        values[datum_name] = convert_str(datum_value)
                    else:
                        values = convert_str(data)

                # put it in self.data
                self.data.append((timestamp, name, values))

            index = end_index + 1


def _get_txt_map(file_name, directory):
    """
    Parse a map from a text file

    format:
    long, lat
    -71.42083704471588, 42.42732027318888
    -71.42078474164009, 42.42732819250417
    ...
    """
    with open(directory + file_name, 'r') as map_file:
        contents = map_file.read()

    split = contents.splitlines()
    header = split.pop(0).split(",")
    lat_index = header.index('lat')
    long_index = header.index('long')

    gps_map = []
    for line in split:
        line_data = line.split(",")
        if len(line_data) == 2:
            lat, long = float(line_data[lat_index]), \
                        float(line_data[long_index])

            gps_map.append((lat, long))

    return gps_map


def _get_gpx_map(file_name, directory):
    """Parse a map from a GPX file"""
    with open(directory + file_name, 'r') as gpx_file:
        contents = gpx_file.read()

    gps_map = []

    # xml parsing. Extract the long and lat from the file
    start_flags = ['<rtept', '<trkpt']
    data_start = None
    for flag in start_flags:
        if contents.find(flag) != -1:
            data_start = flag
            break
    if data_start is None:
        raise ValueError("Invalid file format! Start flag not found...")

    start_index = contents.find(data_start) + len(data_start)
    for line in contents[start_index:].splitlines():
        line = line.strip(" ")
        unparsed = line.split(" ")

        if len(unparsed) > 1:
            if len(unparsed) == 2:
                lat_unparsed, long_unparsed = unparsed
            else:
                _, lat_unparsed, long_unparsed = unparsed

            lat = float(lat_unparsed[5:-1])
            long = float(long_unparsed[5:-10])

            gps_map.append((lat, long))

    return gps_map


def get_map(file_name, directory=None):
    """
    Get a map as a list of tuples [(long0, lat0), (long1, lat1), ...].

    Two possible file types for maps are txt and gpx. You will either need
    to specify the :gpx or :maps directory, or give a file extension for the
    file name. If no directory and no file extension is given, gpx is assumed
    """

    if file_name.endswith('gpx'):
        file_type = "gpx"
    elif file_name.endswith(log_file_type):
        file_type = log_file_type
    else:
        raise ValueError("Invalid file extension: %s" % file_name)

    if directory is None:
        directory = ":maps"

    if file_type == "gpx":
        directory = project.interpret_dir(directory)
        file_name = project.get_file_name(file_name, directory, 'gpx')
        gps_map = _get_gpx_map(file_name, directory)
    else:
        directory = project.interpret_dir(directory)
        file_name = project.get_file_name(file_name, directory, log_file_type)
        gps_map = _get_txt_map(file_name, directory)

    print("Using map named %s, length %i" % (file_name, len(gps_map)))

    return gps_map


def parse_arguments(default_file=-1, default_directory=-1):
    file_name = default_file
    directory = default_directory

    if len(sys.argv) == 2:
        file_name = sys.argv[1]
    elif len(sys.argv) == 3:
        file_name, directory = sys.argv[1:]

    try:
        file_name = int(file_name)
        directory = int(directory)
    except ValueError:
        pass

    return file_name, directory
