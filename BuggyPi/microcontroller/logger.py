import os
import sys
import time
from datetime import datetime
import random

sys.path.insert(0, '../')

import project

time_name_sep = ":\t"
name_values_sep = ";\t"
values_sep = "|\t"
datum_sep = ",\t"

obsolete_data = "Jun 22 2016"


class Logger:
    def __init__(self, file_name, directory):
        if file_name is None or file_name.replace(".txt", "") == "":
            file_name = time.strftime("%c").replace(":", ";") + ".txt"
        elif len(file_name) < 4 or file_name[-4:] != ".txt":
            file_name += ".txt"

        if directory is None:
            directory = project.get_dir(":logs")
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = project.get_dir(":logs") + directory

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", directory + file_name)

        self.time0 = time.time()
        self.log_start = self.time0

        self.data_file = open(directory + file_name, 'w+')

        self.queue = []

    def enq(self, value_name, data):
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

    is_float = False
    for char in string:
        if char == ".":
            is_float = True

        elif not char.isdigit() and char != '-':
            return string
    if is_float:
        return float(string)
    else:
        return int(string)


def get_dir_name(directory):
    if directory is None:
        directory = project.get_dir(":logs")
    elif directory == ":random":
        directories = []
        for local_dir in os.listdir(project.get_dir(":logs")):
            directory = project.get_dir(":logs") + local_dir
            if os.path.isdir(directory):
                directories.append(directory)
        directory = random.choice(directories)
        print("Using directory '%s'" % directory)
    elif os.path.isdir(project.get_dir(":logs") + directory):
        directory = project.get_dir(":logs") + directory
    if directory[-1] != "/":
        directory += "/"
    return directory


def get_files(directory):
    log_files = []
    files = sorted(os.listdir(directory))
    for file in files:
        if len(file) >= 4 and file[-4:] == '.txt':
            log_files.append(file)
    return log_files


def get_file_name(file_name, directory):
    if type(file_name) == int:
        # file_name is the index in the list of files in the directory
        file_name = get_files(directory)[file_name]
    elif type(file_name) == str:
        if file_name == ":random":
            file_name = random.choice(get_files(directory))
        elif len(file_name) < 4 or file_name[-4:] != '.txt':
            file_name += '.txt'
    else:
        raise ValueError("Invalid file name: " + str(file_name))

    return file_name


class Parser:
    def __init__(self, file_name, directory=None):
        self.directory = get_dir_name(directory)
        self.local_dir = directory[self.directory.rfind("/", 0, -1) + 1:]
        self.file_name = get_file_name(file_name, directory)

        print("Using file named '%s'" % self.file_name)

        with open(self.directory + self.file_name, 'r') as data_file:
            self.contents = data_file.read()

        try:
            if (datetime.strptime(directory.split("/")[-2], '%b %d %Y') <=
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
            time_index = self.contents.find(time_name_sep, index)
            timestamp = float(self.contents[index: time_index])

            name_index = self.contents.find(name_values_sep, index)
            name = self.contents[time_index + len(time_name_sep): name_index]

            end_index = self.contents.find("\n", index)
            value_content = self.contents[
                            name_index + len(name_values_sep):end_index]

            values = {}
            for data in value_content.split(values_sep):
                if datum_sep in data:
                    datum_name, datum_value = data.split(datum_sep)
                    values[datum_name] = convert_str(datum_value)
                else:
                    values[None] = convert_str(data)

            index = end_index + 1

            self.data.append((timestamp, name, values))

            if name not in self.initial_values.keys():
                self.initial_values[name] = timestamp, len(self.data), values


def get_map(file_name, directory=":maps"):
    directory = project.get_dir(directory)
    file_name = get_file_name(file_name, directory)
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
