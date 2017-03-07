import sys
import os
import gzip
from atlasbuggy.files.atlasbuggyfile import AtlasReadFile

os.chdir("..")

def rezip_file():
    file_name = None
    directory = None

    if len(sys.argv) > 1:
        file_name = sys.argv[1]
    if len(sys.argv) > 2:
        file_name = sys.argv[1]
        directory = sys.argv[2]

    if file_name is None:
        return
    atlas_file = AtlasReadFile(file_name, directory, False, "txt", "logs")

    print(atlas_file.full_path)
    with open(atlas_file.full_path, "r") as file:
        raw_contents = file.read().encode("utf-8")

    compressed_data = gzip.compress(raw_contents)

    extension_index = atlas_file.file_name.rfind(".")
    new_name = atlas_file.file_name[:extension_index]
    with open(os.path.join(atlas_file.directory, new_name + ".gzip"), "wb") as file:
        file.write(compressed_data)

rezip_file()