"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Allows for easy sensor and command data logging.

This class integrates nicely with the Sensor and Command objects found in
data.py.

Recorder will take in the input object's data and write it to a csv file.
It is formatted to be user-friendly and easy to interpret.

Please refer to objects.py for proper usage tips.
"""

import csv
import os
import sys
import time

import numpy

sys.path.insert(0, '../')

import config


class Recorder(object):
    def __init__(self, headers, frequency=None, file_name=None, directory=None):
        if directory is None:
            self.directory = config.get_dir(":logs")
        else:
            if directory[-1] != "/":
                directory += "/"
            self.directory = directory
            if not os.path.isdir(self.directory):
                self.directory = config.get_dir(":logs") + self.directory

        if file_name is None:
            self.file_name = time.strftime("%c").replace(":", ";") + ".csv"
        else:
            if file_name[-4:] != ".csv":
                file_name += ".csv"
            self.file_name = file_name

        if not os.path.exists(self.directory):
            os.makedirs(self.directory)
        print("Writing to:", self.directory + self.file_name)
        self.csv_file = open(self.directory + self.file_name, 'w+')

        self.writer = csv.writer(self.csv_file, delimiter=',',
                                 quotechar='|',
                                 quoting=csv.QUOTE_MINIMAL)

        self.current_row = []

        self.time0 = time.time()
        self.log_start = self.time0
        self.frequency = frequency
        self.enable_record = True

        # self.row_length = None
        self.writer.writerow(["time"] + headers)
        self.row_length = len(headers)

    def add_row(self, data):
        # if self.row_length is None:
        #     self.writer.writerow(["time"] + list(data.keys()))
        #     self.row_length = len(data)
        if (self.frequency is None) or (
                time.time() - self.time0) > self.frequency:
            self.time0 = time.time()
            self.enable_record = True
        if self.enable_record:
            assert len(data) == self.row_length
            self.writer.writerow([time.time() - self.log_start] + data)#list(data.values()))
            self.enable_record = False
            # print(data)

        # return not self.enable_record

    def close(self):
        self.csv_file.close()


def is_float(string):
    try:
        float(string)
        return True
    except ValueError:
        return False


def parse(file_dir, np_array=True, omit_header_rows=True,
          remove_timestamps=False):
    if not os.path.isdir(file_dir) and not os.path.isfile(file_dir):
        file_dir = config.get_dir(":logs") + file_dir

    with open(file_dir, 'r') as csv_file:
        data = []
        reader = csv.reader(csv_file, delimiter=',',
                            quotechar='|',
                            quoting=csv.QUOTE_MINIMAL)
        for row in reader:
            parsed_row = []
            for datum in row:
                if is_float(datum):
                    parsed_row.append(float(datum))
                elif datum.isdigit():
                    parsed_row.append(int(datum))
                elif datum == "True":
                    parsed_row.append(True)
                elif datum == "False":
                    parsed_row.append(False)
                else:
                    parsed_row.append(datum)

            data.append(parsed_row)
    if omit_header_rows:
        data.pop(0)
        data.pop(0)

    if np_array:
        data = numpy.array(data)

    if remove_timestamps:
        if not np_array:
            data = numpy.array(data)
        data = data[:, 1:]
        if not np_array:
            data = data.tolist()

    return data
