import os
import sys
import time
import re

sys.path.insert(0, '../')

import config

time_name_sep = ":\t"
name_values_sep = ";\t"
values_sep = "|\t"
datum_sep = ",\t"

class Logger:
    def __init__(self, file_name, directory):
        if file_name is None or file_name.replace(".txt", "") == "":
            file_name = time.strftime("%c").replace(":", ";") + ".txt"
        elif len(file_name) < 4 or file_name[-4:] != ".txt":
            file_name += ".txt"
        
        if directory is None:
            directory = config.get_dir(":logs")
        else:
            if directory[-1] != "/":
                directory += "/"
            if not os.path.isdir(directory):
                directory = config.get_dir(":logs") + directory

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", directory + file_name)

        self.time0 = time.time()
        self.log_start = self.time0

        self.data_file = open(directory + file_name, 'w+')

        self.queue = []

    def enq(self, value_name, data):
        self.queue.append((value_name, data))
    
    def record(self):
        while len(self.queue) > 0:
            value_name, data = self.queue.pop(0)
            line = str(time.time() - self.time0) + time_name_sep + str(value_name) + name_values_sep
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

        elif not char.isdigit():
            return string
    if is_float:
        return float(string)
    else:
        return int(string)

class Parser:
    def __init__(self, file_name, directory=None):
        if directory is None:
            directory = config.get_dir(":logs")
        elif os.path.isdir(config.get_dir(":logs") + directory):
            directory = config.get_dir(":logs") + directory
        if directory[-1] != "/":
            directory += "/"

        if len(file_name) < 4 or file_name[-4:] != '.txt':
            file_name += '.txt'
        with open(directory + file_name, 'r') as data_file:
            self.contents = data_file.read()

        self.data = []
        self.iter_index = 0

    def __iter__(self):
        return self

    def __next__(self):
        if self.iter_index < len(self.contents):
            time_index = self.contents.find(time_name_sep, self.iter_index)
            timestamp = float(self.contents[self.iter_index: time_index])

            name_index = self.contents.find(name_values_sep, self.iter_index)
            name = self.contents[time_index + len(time_name_sep): name_index]

            end_index = self.contents.find("\n", self.iter_index)
            value_content = self.contents[name_index + len(name_values_sep):end_index]

            values = {}
            for data in value_content.split(values_sep):
                if datum_sep in data:
                    datum_name, datum_value = data.split(datum_sep)
                    values[datum_name] = convert_str(datum_value)
                else:
                    values['value'] = convert_str(data)

            self.iter_index = end_index + 1

            return timestamp, name, values
        else:
            raise StopIteration
        
if __name__ == '__main__':
    log = Parser("Mon Jun 13 21;17;24 2016")
    for data in log:
        if data[1] == 'encoder':
            print(data)
