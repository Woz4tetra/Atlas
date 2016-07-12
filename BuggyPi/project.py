"""
Written by Ben Warwick

diagnose_system.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Contains functions that return important project directories
"""

import os
import sys

CONFIGDIR = os.path.dirname(os.path.realpath(__file__))


def _arduino_dir():
    root_dir_name = "Atlas"
    root_dir = CONFIGDIR[:CONFIGDIR.rfind(root_dir_name) + len(root_dir_name)]
    return root_dir + "/Arduino/"


directories = {
    'arduino': _arduino_dir(),
    'microcontroller': CONFIGDIR + "/microcontroller/",
    'logs': CONFIGDIR + "/microcontroller/logs/",
    'maps': CONFIGDIR + "/microcontroller/maps/",
    'gpx': CONFIGDIR + "/microcontroller/maps/gpx/",
    'vision': CONFIGDIR + "/vision/",
    'videos': CONFIGDIR + "/vision/videos/",
    'images': CONFIGDIR + "/vision/images/",
    'scripts': CONFIGDIR + "/scripts/",
    'test': CONFIGDIR + "/tests/",
    'tunes': CONFIGDIR + "/sound/tunes/",
    'project': CONFIGDIR + "/",
}


def get_platform():
    if sys.platform.startswith('darwin'):  # OS X
        return "mac"
    elif (sys.platform.startswith('linux') or sys.platform.startswith(
            'cygwin')):
        return "linux"
    elif sys.platform.startswith('win'):  # Windows
        return "win"
    else:
        return None


def get_dir(directory=""):
    abs_directory = ""
    if len(directory) > 0 and directory[0] == ':':
        shortcut_start = directory.find(":") + 1
        shortcut_end = directory.find("/", shortcut_start)
        if shortcut_end == -1:
            key = directory[shortcut_start:]
            abs_directory = directories[key]
        else:
            key = directory[shortcut_start: shortcut_end]
            abs_directory = directories[key] + directory[shortcut_end + 1:]

    elif len(directory) > 0 and directory[0] == '/':
        abs_directory = directory

    else:
        abs_directory = CONFIGDIR + "/" + directory

    if not os.path.isdir(abs_directory):
        os.mkdir(abs_directory)

    return abs_directory
