"""
    Written by Ben Warwick
    
    map_maker.py, written for the Self-Driving Buggy Project
    Version 11/24/2015
    =========
    
    Converts gpx files to csv. If we collect with a map gps that's not on the
    arduino, we will convert its data with this script
"""

import sys

sys.path.insert(0, '../')

import config
from controller import map


def convert_gpx(directory):
    directory = config.get_dir(directory)
    with open(directory, 'r') as gpx_file:
        contents = gpx_file.read()
        data = []

        while len(contents) > 2:
            lat_index_start = contents.find("lat") + 5
            lat_index_end = contents.find('"', lat_index_start)
            latitude = contents[lat_index_start: lat_index_end]

            contents = contents[lat_index_end:]

            lon_index_start = contents.find("lon") + 5
            lon_index_end = contents.find('"', lon_index_start)
            longitude = contents[lon_index_start: lon_index_end]

            data.append([latitude, longitude])

            contents = contents[lon_index_end:]

        data.pop(-1)
        map.write_map(data, directory[:-4])


if __name__ == '__main__':
    arguments = sys.argv

    convert_gpx(arguments[1])
