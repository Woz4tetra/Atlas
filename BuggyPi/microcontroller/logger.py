import os
import sys
from datetime import datetime

sys.path.insert(0, '../')

import directories

time_name_sep = ":\t"
name_values_sep = ";\t"
values_sep = "|\t"
datum_sep = ",\t"

obsolete_data = "Jun 26 2016"


class Logger:
    def __init__(self, file_name, directory):
        if file_name is None or file_name.replace(".txt", "") == "":
            file_name = time.strftime("%c").replace(":", ";") + ".txt"
        elif len(file_name) < 4 or file_name[-4:] != ".txt":
            file_name += ".txt"

        if directory is None:
            directory = directories.get_dir(":logs")
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = directories.get_dir(":logs") + directory

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


class Parser:
    def __init__(self, file_name, directory=None):
        if directory is None:
            directory = directories.get_dir(":logs")
        elif os.path.isdir(directories.get_dir(":logs") + directory):
            directory = directories.get_dir(":logs") + directory
        if directory[-1] != "/":
            directory += "/"

        if type(file_name) == str:
            if len(file_name) < 4 or file_name[-4:] != '.txt':
                file_name += '.txt'
        elif type(file_name) == int:
            files = sorted(os.listdir(directory))
            log_files = []
            for file in files:
                if len(file) >= 4 and file[-4:] == '.txt':
                    log_files.append(file)
            # file_name is the index in the list of files in the directory
            file_name = log_files[file_name]
            print("Using file named '%s'" % file_name)
        else:
            raise ValueError("Invalid file name: " + str(file_name))
        with open(directory + file_name, 'r') as data_file:
            self.contents = data_file.read()

        try:
            if (datetime.strptime(file_name, '%b %d %Y') <=
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

        self.initials = {}

        self.create_data()

    def __iter__(self):
        return self

    def __next__(self):
        if self.iter_index < len(self.data):
            datum = self.data[self.iter_index]
            self.iter_index += 1
            return datum
        else:
            raise StopIteration

    def __getitem__(self, item):
        return self.data[item]

    def __len__(self):
        return len(self.data)

    def get_first(self, name):
        return self.initials[name]

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

            if name not in self.initials.keys():
                self.initials[name] = timestamp, len(self.data), values


def get_points(file_name="checkpoints.txt", directory=":logs"):
    directory = directories.get_dir(directory)
    with open(directory + file_name, 'r') as checkpoints:
        contents = checkpoints.read()

    split = contents.splitlines()
    header = split.pop(0).split(",")

    if header[1] == 'lat':
        lat_index = 1
        long_index = 2
    else:
        lat_index = 2
        long_index = 1

    checkpoints = []
    for line in split:
        line_data = line.split(",")
        if len(line_data) == 3:
            num = int(line_data[0])
            while len(checkpoints) < num + 1:
                checkpoints.append(None)
            long, lat = float(line_data[long_index]), float(
                line_data[lat_index])

            checkpoints[num] = long, lat

    return checkpoints


if __name__ == '__main__':
    log = Parser("Mon Jun 13 21;23;34 2016", "Jun 13 2016")
    import time

    time0 = time.time()
    for log_data in log:
        if log_data[1] == 'gps':
            print(log_data)
    print(time.time() - time0)
    print(get_points())
