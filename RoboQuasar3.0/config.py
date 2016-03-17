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
    'logs': CONFIGDIR + "/analyzers/logs/",
    'maps': CONFIGDIR + "/analyzers/maps/",
    'gpx': CONFIGDIR + "/analyzers/maps/gpx/",
    'camera': CONFIGDIR + "/camera/",
    'videos': CONFIGDIR + "/camera/Videos/",
    'images': CONFIGDIR + "/camera/Images/",
    'controllers': CONFIGDIR + "/controllers/",
    'scripts': CONFIGDIR + "/scripts/",
    'test': CONFIGDIR + "/tests/",
    'sound': CONFIGDIR + "/sounds/",
    'tunes': CONFIGDIR + "/sounds/tunes/",
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
    if len(directory) > 0 and directory[0] == ':':
        shortcut_start = directory.find(":") + 1
        shortcut_end = directory.find("/", shortcut_start)
        if shortcut_end == -1:
            key = directory[shortcut_start:]
            return directories[key]
        else:
            key = directory[shortcut_start: shortcut_end]
            return directories[key] + directory[shortcut_end + 1:]

    elif len(directory) > 0 and directory[0] == '/':
        return directory

    else:
        return CONFIGDIR + "/" + directory
