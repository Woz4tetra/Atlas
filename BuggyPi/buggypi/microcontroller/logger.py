import os
import sys
import time
from datetime import datetime

from buggypi import project

time_name_sep = ":\t"
name_values_sep = ";\t"
values_sep = "|\t"
datum_sep = ",\t"

obsolete_data = "Jun 22 2016"
log_file_type = "txt"
log_directory = ":logs"


def log_folder():
    month = time.strftime("%b")
    day = time.strftime("%d")
    year = time.strftime("%Y")
    return "%s %s %s/" % (month, day, year)

class Logger:
    def __init__(self, file_name, directory):
        if file_name is None or file_name.replace("." + log_file_type,
                                                  "") == "":
            file_name = time.strftime("%c").replace(":",
                                                    ";") + "." + log_file_type
        elif len(file_name) < 4 or file_name[-4:] != "." + log_file_type:
            file_name += "." + log_file_type

        if directory == ":today":
            directory = project.get_dir(log_directory) + log_folder()
        elif directory is None:
            directory = project.get_dir(log_directory)
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = project.get_dir(log_directory) + directory

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", directory + file_name)

        self.time0 = time.time()
        self.log_start = self.time0

        self.data_file = open(directory + file_name, 'w+')

        self.queue = []

    def enq(self, value_name, data, timestamp=None):
        if timestamp is None:
            timestamp = time.time() - self.time0
        self.queue.append((timestamp, value_name, data))

    def record(self):
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
        self.record()
        self.data_file.close()


def convert_str(string):
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
    def __init__(self, file_name, directory=None):
        self.directory = project.parse_dir(directory, ":logs")
        self.local_dir = directory[self.directory.rfind("/", 0, -1) + 1:]
        self.file_name = project.get_file_name(file_name, self.directory,
                                               log_file_type)

        print("Using file named '%s'" % self.file_name)

        with open(self.directory + self.file_name, 'r') as data_file:
            self.contents = data_file.read()

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

        self.data = []
        self.iter_index = 0

        self.initial_values = {}

        self.create_data()

    def __iter__(self):
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

    def get(self, sensor_name, instance_num=0):
        if instance_num == 0:
            return self.initial_values[sensor_name]
        else:
            counter = 0
            for index, (timestamp, name, values) in enumerate(self.data):
                if name == sensor_name:
                    counter += 1
                    if counter == instance_num:
                        return timestamp, index, values

    def create_data(self):
        index = 0
        while index < len(self.contents):
            end_index = self.contents.find("\n", index)
            time_index = self.contents.find(time_name_sep, index, end_index)
            name_index = self.contents.find(name_values_sep, index, end_index)

            if time_index != -1 and name_index != -1 and end_index != -1:
                timestamp = float(self.contents[index: time_index])
                name = self.contents[
                       time_index + len(time_name_sep): name_index]
                value_content = self.contents[
                                name_index + len(name_values_sep):end_index]
                values = {}
                for data in value_content.split(values_sep):
                    if datum_sep in data:
                        datum_name, datum_value = data.split(datum_sep)
                        values[datum_name] = convert_str(datum_value)
                    else:
                        values = convert_str(data)

                self.data.append((timestamp, name, values))

                if name not in self.initial_values.keys():
                    self.initial_values[name] = timestamp, len(
                        self.data), values

            index = end_index + 1


def get_map(file_name, directory=None):
    if directory is None:
        directory = ":maps"
    directory = project.get_dir(directory)
    file_name = project.get_file_name(file_name, directory, log_file_type)
    with open(directory + file_name, 'r') as map_file:
        contents = map_file.read()

    split = contents.splitlines()
    header = split.pop(0).split(",")

    if header[0] == 'lat':
        lat_index = 0
        long_index = 1
    else:
        lat_index = 1
        long_index = 0

    gps_map = []
    for line in split:
        line_data = line.split(",")
        if len(line_data) == 2:
            long, lat = float(line_data[long_index]), float(
                line_data[lat_index])

            gps_map.append((long, lat))

    return gps_map
