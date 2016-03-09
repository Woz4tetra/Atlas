
import pyb
import os
import time

class Recorder(object):
    def __init__(self, frequency=None, file_name=None, directory=None):
        if directory is None:
            if os.path.exists("/sd"):
                self.directory = "/sd/logs/"
            else:
                self.directory = "/flash/logs/"
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

        self.current_row = ["timestamp"]
        self.sensor_indices = {}
        self.header_row = []

        self.time0 = pyb.millis()
        self.log_init_time = self.time0
        self.frequency = frequency
        self.enable_record = True

    def write_row(data_list):
        format_str = '{},' * len(data_list)
        format_str = format_str[:-1] + "\n"
        self.csv_file.write(format_str.format(*data_list))

    def add_tracker(self, sensor, name):
        if sensor.log_names is not None:
            self.header_row.append(
                    (sensor.object_id, name, sensor, sensor.log_names))
        else:
            raise ValueError(
                "Serial object does not have data points to log. "
                "Please supply this object's log_names parameter with a list "
                "of strings."
            )

    def end_init(self):
        self.header_row.sort(key=lambda element: element[0])
        names_row = [""]
        for sensor_info in self.header_row:
            object_id, sensor_name, sensor, data_names = sensor_info

            self.sensor_indices[object_id] = len(self.current_row)

            names_row.append(sensor_name)
            names_row += [""] * (len(data_names) - 1)
            self.current_row += data_names
        self.write_row(names_row)
        self.write_row(self.current_row)

    def add_data(self, serial_object):
        if self.enable_record:
            start_index = self.sensor_indices[serial_object.object_id]
            data = serial_object.update_log()
            if len(data) != len(serial_object.log_names):
                raise ValueError(
                    "Data provided by update_log does not match expected number"
                    "of data points provided by log_names."
                )
            for index in range(len(data)):
                self.current_row[index + start_index] = data[index]

    def end_row(self):
        if self.enable_record:
            self.current_row[0] = pyb.millis() - self.log_init_time
            self.write_row(self.current_row)
            self.enable_record = False

        if (self.frequency is None) or (pyb.millis() - self.time0) > self.frequency:
            self.time0 = pyb.millis()
            self.enable_record = True

    def close(self):
        self.csv_file.close()
