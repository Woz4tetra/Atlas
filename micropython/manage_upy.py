import os
import sys
import pwd
import shutil
import errno

from autobuggy import project


def get_default_sd_name():
    with open("default_sd.txt", 'r') as default_file:
        name = default_file.read()
    return name


def set_default_sd_name(name):
    with open("default_sd.txt", 'w+') as default_file:
        default_file.write(name)

def remove_directory(directory):
    shutil.rmtree(directory)

def copy_files(src, dest):
    try:
        shutil.copytree(src, dest)
    except OSError as e:
        # If the error was caused because the source wasn't a directory
        if e.errno == errno.ENOTDIR:
            shutil.copy(src, dest)
#        else:
#            print('Directory not copied. Error: %s' % e)

default_sd_name = get_default_sd_name()

def get_username():
    return pwd.getpwuid(os.getuid())[0]


def get_upy_sd_dir(upy_sd_name):
    platform = project.get_platform()
    if platform == "mac":
        return os.path.join("/Volumes", upy_sd_name)
    elif platform == "linux":
        return os.path.join("/media", upy_sd_name)
    else:
        raise NotImplementedError


def update_upy(upy_sd_name, upy_sd_dir=None):
    if upy_sd_dir is None:
        upy_sd_dir = get_upy_sd_dir(upy_sd_name)

    backup_dir = os.path.join(project.root_dir, "MicroPython/backups")
    main_dir = os.path.join(project.root_dir, "MicroPython/PYCARD")

    if os.path.isdir(upy_sd_dir):
        # delete everything in backup directory
        if os.path.isdir(backup_dir):
            remove_directory(backup_dir)
        os.mkdir(backup_dir)
        print("backup dir removed")

        # backup the current PYCARD files
        for f in os.listdir(upy_sd_dir):
            if f[0] != ".":
                sub_path = os.path.join(upy_sd_dir, f)
                copy_files(sub_path, os.path.join(backup_dir, f))
        print("moved SD card content to backup")
        
        # remove everything in SD card
        for f in os.listdir(upy_sd_dir):
            if f[0] != ".":
                sub_path = os.path.join(upy_sd_dir, f)
                if os.path.isdir(sub_path):
                    # if errors happen, ignore the file
                    shutil.rmtree(sub_path, onerror=lambda *x: x)
                else:
                    os.remove(sub_path)
        print("deleted SD card content")
        
        # copy files in MicroPython/PYCARD to SD card
        for f in os.listdir(main_dir):
            if f[0] != ".":
                sub_path = os.path.join(main_dir, f)
                copy_files(sub_path, os.path.join(upy_sd_dir, f))
        print("copied local PYCARD files to SD card")
    else:
        raise FileNotFoundError("MicroPython SD card not found...")

if __name__ == '__main__':
    if len(sys.argv) > 1:
        upy_sd_name = sys.argv[1]
        set_default_sd_name(upy_sd_name)
    else:
        upy_sd_name = default_sd_name
    print("SD card name:", upy_sd_name)

    update_upy(upy_sd_name)
