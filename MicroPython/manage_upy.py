import os
import sys
import pwd
import shutil
import errno

from autobuggy import project

default_sd_name = get_default_sd_name()

def copy_anything(src, dst):
    try:
        shutil.copytree(src, dst)
    except OSError as exc:  # python >2.5
        if exc.errno == errno.ENOTDIR:
            shutil.copy(src, dst)
        else:
            raise


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


def get_default_sd_name():
    with open("default_sd.txt", 'r') as default_file:
        name = default_file.read()
    return name


def set_default_sd_name(name):
    with open("default_sd.txt", 'w+') as default_file:
        default_file.write(name)


def update_upy(upy_sd_name, upy_sd_dir=None):
    if upy_sd_dir is None:
        upy_sd_dir = get_upy_sd_dir(upy_sd_name)

    backup_dir = os.path.join(project.root_dir, "MicroPython/backups")
    main_dir = os.path.join(project.root_dir, "MicroPython/PYCARD")

    if os.path.isdir(upy_sd_dir):
        if os.path.isdir(backup_dir):
            shutil.rmtree(backup_dir)
        copy_anything(upy_sd_dir, backup_dir)
        for f in os.listdir(upy_sd_dir):
            abs_path = os.path.join(upy_sd_dir, f)
            if os.path.isdir(abs_path):
                shutil.rmtree(abs_path)
            else:
                os.remove(abs_path)

        for f in os.listdir(main_dir):
            abs_path = os.path.join(main_dir, f)
            if os.path.isdir(abs_path):
                copy_anything(abs_path, os.path.join(upy_sd_dir, f))
            else:
                copy_anything(abs_path, upy_sd_dir)
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
