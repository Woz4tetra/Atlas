"""
Written by Ben Warwick

diagnose_system.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Contains functions that return important project directories
"""

import os
import random
import sys

PROJECTDIR = os.path.dirname(os.path.realpath(__file__))


def _arduino_dir():
    root_dir_name = "Atlas"
    root_dir = PROJECTDIR[:PROJECTDIR.rfind(root_dir_name) + len(root_dir_name)]
    return root_dir + "/Arduino/"


project_dirs = {
    'arduino': _arduino_dir(),
    'microcontroller': PROJECTDIR + "/microcontroller/",
    'logs': PROJECTDIR + "/logs/",
    'maps': PROJECTDIR + "/maps/",
    'gpx': PROJECTDIR + "/maps/gpx/",
    'vision': PROJECTDIR + "/vision/",
    'videos': PROJECTDIR + "/vision/videos/",
    'images': PROJECTDIR + "/vision/images/",
    'scripts': PROJECTDIR + "/scripts/",
    'test': PROJECTDIR + "/tests/",
    'tunes': PROJECTDIR + "/sound/tunes/",
    'project': PROJECTDIR + "/",
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
            abs_directory = project_dirs[key]
        else:
            key = directory[shortcut_start: shortcut_end]
            abs_directory = project_dirs[key] + directory[shortcut_end + 1:]

    elif len(directory) > 0 and directory[0] == '/':
        abs_directory = directory

    else:
        abs_directory = PROJECTDIR + "/" + directory

    if not os.path.isdir(abs_directory):
        os.mkdir(abs_directory)

    return abs_directory


def parse_dir(directory, default):
    """
    Takes an directory name and converts it to an absolute directory
    """
    if directory is None:
        directory = get_dir(default)
    elif directory == ":random":
        directories = []
        for local_dir in os.listdir(get_dir(default)):
            directory = get_dir(default) + local_dir
            if os.path.isdir(directory):
                directories.append(directory)
        directory = random.choice(directories)
        print("Using directory '%s'" % directory)
    elif os.path.isdir(get_dir(default) + directory):
        directory = get_dir(default) + directory
    if directory[-1] != "/":
        directory += "/"
    return directory


def _get_files(directory, file_types):
    """
    Gets all file names of the specified file type in a directory
    """
    if type(file_types) == str:
        file_types = [file_types]
    log_files = []
    files = sorted(os.listdir(directory))
    for file in files:
        for file_type in file_types:
            if file.endswith(file_type):
                log_files.append(file)
    return log_files


def get_file_name(file_name, directory, file_types):
    """
    Gets a file within a directory. The file name can be the index of the file
    in the directory ordered by name, the name of the file, or a random file
    (specified by suppling :random for file_name.

    file_type is the desired file extension (for example: 'txt', 'avi')
        don't put in the '.' before the extension!!
    """
    if type(file_name) == int:
        # file_name is the index in the list of files in the directory
        file_name = _get_files(directory, file_types)[file_name]
    elif type(file_name) == str:
        if file_name == ":random":
            file_name = random.choice(_get_files(directory, file_types))
        elif type(file_types) == str:
            if not file_name.endswith(file_types):
                file_name += '.' + file_types
    else:
        raise ValueError("Invalid file name: " + str(file_name))

    return file_name
