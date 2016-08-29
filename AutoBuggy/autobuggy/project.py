"""
Contains functions that return important project directories
"""

import os
import random
import sys

autobuggy_dir = os.path.dirname(os.path.realpath(__file__))
project_dir = autobuggy_dir

ROOT_DIR_NAME = "Atlas"
ROOT_DIR = autobuggy_dir[:autobuggy_dir.rfind(ROOT_DIR_NAME) + len(ROOT_DIR_NAME)]

# dictionary of important local project directories
project_dirs = {
    'logs': "logs/",
    'maps': "maps/",
    'pickled': "pickled/",
    'gpx': "maps/gpx/",
    'videos': "videos/",
    'images': "images/",
    'joysticks': "joysticks/",
    'project': "",
}


def set_project_dir(project_name=None):
    """
    Sets the project directory that project_dirs should use. One benefit of this
    is each project (or robot) can have its own set of log files and maps
    """
    global project_dir
    if project_name is not None:  # if None, use the AutoBuggy project directory
        # walk through all directories from the top down until the project name
        # is found
        for root, dirs, files in os.walk(ROOT_DIR):
            if project_name in dirs:
                project_dir = os.path.join(root, project_name)
                break
    print("Project directory is", project_dir)

    # update project_dirs
    for name, directory in project_dirs.items():
        project_dirs[name] = os.path.join(project_dir, directory)

    return project_dir


def add_project_dirs(**new_project_dirs):
    """Add any special project directories"""
    for name, local_dir in project_dirs.items():
        if new_project_dirs[name][0] != "/":
            project_dirs[name] = os.path.join(project_dir, new_project_dirs[name])
        else:
            project_dirs[name] = os.path.join(project_dir, new_project_dirs[name])

        if project_dirs[name][-1] != "/":
            project_dirs[name][-1] += "/"


def get_platform():
    """Use for platform specific operations"""
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
    """
    Search project_dirs and format the directory into an absolute
    directory. If the directory doesn't exist, make it
    """
    if len(directory) > 0 and directory[0] == ':':
        shortcut_start = directory.find(":") + 1
        shortcut_end = directory.find("/", shortcut_start)
        if shortcut_end == -1:
            key = directory[shortcut_start:]
            abs_directory = project_dirs[key]
        else:
            key = directory[shortcut_start: shortcut_end]
            abs_directory = os.path.join(project_dirs[key], directory[shortcut_end + 1:])

    elif len(directory) > 0 and directory[0] == '/':
        abs_directory = directory
    else:
        abs_directory = os.path.join(autobuggy_dir, directory) + "/"

    if abs_directory[0:2] == "--":
        raise NotADirectoryError(
            "Make sure to set your project name. It must be a directory inside "
            "the Atlas github repository: project.set_project_dir"
            "('project name')")
    if not os.path.isdir(abs_directory):
        os.mkdir(abs_directory)

    return abs_directory


def parse_dir(directory, default):
    """
    Formats the input directory using get_dir. The 'default' argument is the
    directory to start in. It should be a project directory flag. Useful if
    the user doesn't provide a specific directory but you know what kind of
    file in the project you are looking for.

    If :random is provided for the directory, a directory will selected at
    random from within the default directory
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
    file_types can be a list of file types or a string containing one file type
    """
    if type(file_types) == str:
        file_types = [file_types]
    log_files = []
    files = sorted(os.listdir(directory), key=lambda v: v.lower())
    for file in files:
        for file_type in file_types:
            if file.endswith(file_type):
                log_files.append(file)
    return log_files


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
