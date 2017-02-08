"""
Contains functions that return important project directories
"""

import os
import sys
import re
import random

# dictionary of important local project directories
project_dirs = {
    'logs'     : "logs/",
    'maps'     : "maps/",
    # 'pickled'    : "pickled/",
    'gpx'      : "maps/gpx/",
    'videos'   : "videos/",
    'images'   : "images/",
    'joysticks': "joysticks/",
    # 'simulations': "pickled/simulations/",
    'project'  : "",
}


def get_platform():
    """Use for platform specific operations"""
    if sys.platform.startswith('darwin'):  # OS X
        return "mac"
    elif (sys.platform.startswith('linux') or sys.platform.startswith(
            'cygwin')):
        return "linux"
    elif sys.platform.startswith('win'):  # Windows
        return "windows"
    else:
        return None


def interpret_dir(directory="."):
    """
    Search project_dirs and format the directory into an absolute
    directory. If the directory doesn't exist, make it

    A directory shortcut starts with ":"
    For example:
        ":logs" becomes "[full path to]/logs/"
        ":logs/Dec 27 2016" becomes "[full path to]/logs/Dec 27 2016/"

    :return: Absolute path to directory
    """
    if len(directory) > 0 and directory[0] == ':':
        shortcut_start = directory.find(":") + 1
        shortcut_end = directory.find("/", shortcut_start)
        if shortcut_end == -1:
            key = directory[shortcut_start:]
            directory = project_dirs[key]
        else:
            key = directory[shortcut_start: shortcut_end]
            directory = os.path.join(project_dirs[key],
                                     directory[shortcut_end + 1:])

    abs_directory = os.path.abspath(directory)

    if not os.path.isdir(abs_directory):
        os.mkdir(abs_directory)

    return abs_directory


def parse_dir(directory, default, sort_fn=None):
    """
    Formats the input directory using get_dir. The 'default' argument is the
    directory to start in. It should be a project directory flag. Useful if
    the user doesn't provide a specific directory but you know what kind of
    file in the project you are looking for.

    If :random is provided for the directory, a directory will selected at
    random from within the default directory
    """
    if directory is None:
        directory = interpret_dir(default)
    elif ":" in directory:
        marker = directory.find(":")
        end_marker = directory.find("/", marker)
        if end_marker == -1:
            end_marker = len(directory)
        if directory[marker + 1:end_marker + 1] == "random":
            directories = []
            for local_dir in os.listdir(interpret_dir(default)):
                directory = os.path.join(interpret_dir(default), local_dir)
                if os.path.isdir(directory):
                    directories.append(directory)
            directory = random.choice(directories)
        else:
            match = re.match(r"[0-9-]+", directory[marker + 1:end_marker])
            if match is not None:
                directory_num = int(match.group(0))
                outer_dir_path = os.path.join(interpret_dir(default), directory[:marker])
                numbered_dir = _get_dirs(interpret_dir(outer_dir_path), sort_fn)[directory_num]
                directory = os.path.join(numbered_dir, directory[end_marker + 1:])

        print("Using directory '%s'" % directory)

    elif type(directory) == int:
        directory = _get_dirs(interpret_dir(default), sort_fn)[directory]

    elif os.path.isdir(os.path.join(interpret_dir(default), directory)):
        directory = os.path.join(interpret_dir(default), directory)

    return directory


def _get_dirs(directory, sort_fn):
    local_dir = []
    directories = []
    contents = sorted(os.listdir(directory), key=lambda v: v.lower())
    for item in contents:
        full_dir = os.path.join(directory, item)
        if os.path.isdir(full_dir):
            local_dir.append(item)
            directories.append(full_dir)

    def internal_sort_fn(
            x):  # hack to just use the first element in the zipped list
        return sort_fn(x[0])

    # sort full directory list in the order the local directory list is sorted
    directories = [full for (local, full) in
                   sorted(zip(local_dir, directories), key=internal_sort_fn)]
    return directories


def _get_files(directory, file_types):
    """
    Gets all file names of the specified file type in a directory
    file_types can be a list of file types or a string containing one file type
    """
    if type(file_types) == str:
        file_types = [file_types]
    file_names = []
    contents = sorted(os.listdir(directory), key=lambda v: v.lower())
    for item in contents:
        for file_type in file_types:
            if item.endswith(file_type):
                file_names.append(item)
    return file_names


def get_file_name(file_name, directory, file_types):
    """
    Gets a file within a directory. The file name can be the index of the file
    in the directory ordered by name, the name of the file, or a random file
    (specified by suppling :random for file_name).

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


def _get_gpx_map(file_name, directory):
    """Parse a map from a GPX file"""
    with open(os.path.join(directory, file_name), 'r') as gpx_file:
        contents = gpx_file.read()

    gps_map = []

    # xml parsing. Extract the long and lat from the file
    start_flags = ['<rtept', '<trkpt']
    data_start = None
    for flag in start_flags:
        if contents.find(flag) != -1:
            data_start = flag
            break
    if data_start is None:
        raise ValueError("Invalid file format! Start flag not found...")

    start_index = contents.find(data_start) + len(data_start)
    for line in contents[start_index:].splitlines():
        line = line.strip(" ")
        unparsed = line.split(" ")

        if len(unparsed) > 1:
            if len(unparsed) == 2:
                lat_unparsed, long_unparsed = unparsed
            else:
                _, lat_unparsed, long_unparsed = unparsed

            lat = float(lat_unparsed[5:-1])
            long = float(long_unparsed[5:-10])

            gps_map.append((lat, long))

    return gps_map


def get_map(file_name, directory=None):
    """
    Get a map as a list of tuples [(long0, lat0), (long1, lat1), ...].

    Two possible file types for maps are txt and gpx. You will either need
    to specify the :gpx or :maps directory, or give a file extension for the
    file name. If no directory and no file extension is given, gpx is assumed
    """

    if directory is None:
        directory = ":maps"

    directory = interpret_dir(directory)
    file_name = get_file_name(file_name, directory, 'gpx')
    gps_map = _get_gpx_map(file_name, directory)

    print("Using map named", file_name)
    print("Length of map is", len(gps_map))

    return gps_map


def parse_arguments(default_file=-1, default_directory=-1):
    file_name = default_file
    directory = default_directory

    if len(sys.argv) == 2:
        file_name = sys.argv[1]
    elif len(sys.argv) == 3:
        file_name, directory = sys.argv[1:]

    try:
        file_name = int(file_name)
        directory = int(directory)
    except ValueError:
        pass

    return file_name, directory
