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
import sys
import time
import os
import numpy

sys.path.insert(0, '../')

import config


class Recorder(object):
    def __init__(self, frequency=None, file_name=None, directory=None):
        if directory is None:
            self.directory = config.get_dir(":logs")
        else:
            if directory[-1] != "/":
                directory += "/"
            self.directory = directory

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

        self.current_row = ["timestamp"]
        self.sensor_indices = {}
        self.header_row = []

        self.time0 = time.time()
        self.log_init_time = self.time0
        self.frequency = frequency
        self.enable_record = True

    def add_tracker(self, sensor, name):
        self.header_row.append(
                (sensor.object_id, name, sensor, sensor._properties.keys()))

    def end_init(self):
        self.header_row.sort(key=lambda element: element[0])
        names_row = [""]
        for sensor_info in self.header_row:
            object_id, sensor_name, sensor, data_names = sensor_info

            self.sensor_indices[object_id] = len(self.current_row)

            names_row.append(sensor_name)
            names_row += [""] * (len(data_names) - 1)
            self.current_row += data_names
        self.writer.writerow(names_row)
        self.writer.writerow(self.current_row)

    def add_data(self, serial_object, new_data_received=True):
        if self.enable_record:
            start_index = self.sensor_indices[serial_object.object_id]
            if new_data_received:
                for index, key in enumerate(serial_object._properties):
                    self.current_row[index + start_index] = \
                        serial_object._properties[key]
            else:
                for index in range(len(serial_object._properties)):
                    self.current_row[index + start_index] = ""


    def end_row(self):
        if self.enable_record:
            self.current_row[0] = time.time() - self.log_init_time
            self.writer.writerow(self.current_row)
            self.enable_record = False

        if (self.frequency is None) or (time.time() - self.time0) > self.frequency:
            self.time0 = time.time()
            self.enable_record = True

    def close(self):
        self.csv_file.close()


def is_float(string):
    try:
        float(string)
        return True
    except ValueError:
        return False

def parse(file_dir, np_array=True, omit_header_rows=True, remove_timestamps=False):
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
                elif datum == "":
                    parsed_row.append(None)
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
