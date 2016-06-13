import os
import sys
import time
import re

sys.path.insert(0, '../')

import config


class Logger:
    def __init__(self, directory, file_name):

        if not os.path.exists(directory):
            os.makedirs(directory)
        print("Writing to:", directory + file_name)
        # self.csv_file = open(self.directory + self.file_name, 'w+')
        #
        # self.writer = csv.writer(self.csv_file, delimiter=',',
        #                          quotechar='|',
        #                          quoting=csv.QUOTE_MINIMAL)

        self.current_row = []

        self.time0 = time.time()
        self.log_start = self.time0

        self.data_file = open(directory + file_name, 'w+')

    def record(self, value_name, data):
        line = "%i:%s;" % (time.time() - self.time0, value_name)
        if isinstance(data, dict):
            for prop_name, value in data.items():
                line += prop_name + ":" + str(value) + ":"
            line = line[:-1] + "\n"
        else:
            line += str(data) + "\n"
        self.data_file.write(line)

    def close(self):
        self.data_file.close()

def parse(log_dir, np_array=True, omit_header_row=True,
          remove_timestamps=False, start=0, stop=None, density=1):
    if not os.path.isdir(log_dir) and not os.path.isfile(log_dir):
        log_dir = config.get_dir(":logs") + log_dir
    if not os.path.isdir(log_dir):
        raise FileNotFoundError("Log could not be found:", log_dir)
    if log_dir[-1] != "/":
        log_dir += "/"

    data = []
    for file_name in os.listdir(log_dir):
        with open(log_dir + file_name, 'r') as data_file:
            contents = data_file.read()

        pattern = re.compile(r"""\|\s*                 # opening bar and whitespace
            '(?P<name>.*?)'       # quoted name
            \s*\|\s*(?P<n1>.*?)   # whitespace, next bar, n1
            \s*\|\s*(?P<n2>.*?)   # whitespace, next bar, n2
            \s*\|""", re.VERBOSE)
        match = pattern.match(contents)


