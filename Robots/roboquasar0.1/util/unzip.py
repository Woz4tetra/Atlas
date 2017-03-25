import sys
import os
import gzip
from atlasbuggy.files.atlasbuggyfile import AtlasReadFile

# os.chdir("..")


def unzip_file():
    file_name = None
    directory = None

    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    if len(sys.argv) > 2:
        file_name = sys.argv[1]
        directory = sys.argv[2]

    if file_name is None:
        return
    atlas_file = AtlasReadFile(file_name, directory, True, "gzip", "logs")
    atlas_file.open()

    extension_index = atlas_file.file_name.rfind(".")
    new_name = atlas_file.file_name[:extension_index]
    with open(os.path.join(atlas_file.directory, new_name + ".txt"), "w+") as file:
        file.write(atlas_file.contents)


unzip_file()
