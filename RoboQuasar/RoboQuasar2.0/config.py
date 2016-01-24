# contains functions that return important project directories

import os

CONFIGDIR = os.path.dirname(os.path.realpath(__file__))


def _arduino_dir():
    root_dir_name = "Self-Driving-Buggy"
    root_dir = CONFIGDIR[:CONFIGDIR.rfind(root_dir_name) + len(root_dir_name)]
    return root_dir + "/Arduino/"


directories = {
    'arduino': _arduino_dir(),
    'board': CONFIGDIR + "/board/",
    'logs': CONFIGDIR + "/board/logs/",
    'camera': CONFIGDIR + "/camera/",
    'videos': CONFIGDIR + "/camera/Videos/",
    'images': CONFIGDIR + "/camera/Images/",
    'controller': CONFIGDIR + "/controller/",
    'maps': CONFIGDIR + "/controller/maps/",
    'gpx': CONFIGDIR + "/controller/maps/gpx/",
    'scripts': CONFIGDIR + "/scripts/",
    'project': CONFIGDIR + "/",
}


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
