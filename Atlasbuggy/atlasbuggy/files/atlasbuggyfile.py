"""
Contains functions that return important project directories
"""

import os
import gzip
import string
from stat import S_ISREG, ST_CTIME, ST_MODE

number_characters = set(string.digits)
whitespace_characters = set(string.whitespace)


class AtlasFile:
    def __init__(self, input_name, input_dir, file_type, default_dir):
        self.input_name = input_name
        self.input_dir = input_dir
        self.file_type = file_type
        self.default_dir = default_dir

        self.directory = self.get_abs_dir(self.input_dir)
        self.file_name = self.get_file_name(input_name, self.directory)
        self.full_path = os.path.join(self.directory, self.file_name)

        self.contents = ""
        self.raw_contents = ""

    def get_abs_dir(self, directory):
        abs_default_dir = os.path.abspath(self.default_dir)

        if directory is None:
            directories = os.listdir(abs_default_dir)
            entries = [os.path.join(abs_default_dir, entry) for entry in directories]
            files = filter(os.path.isdir, entries)
            directory = sorted(files, key=lambda x: os.path.getmtime(x))

            return directory[-1]
        elif directory[0] != "/":
            return os.path.join(abs_default_dir, directory)
        elif directory[0] == "/" and os.path.exists(directory):
            return directory
        else:
            raise NotADirectoryError("Input directory not found! '%s'", self.input_dir)

    def get_file_name(self, file_name, directory):
        if file_name is None:  # retrieve last file
            entries = os.listdir(self.directory)
            entries = [os.path.join(self.directory, entry) for entry in entries]
            files = filter(lambda entry: entry.endswith(self.file_type), entries)
            files = sorted(files, key=lambda x: os.path.getmtime(x))
            if len(files) == 0:
                raise FileNotFoundError(
                    "File of type '%s' not found in directory: %s" % (self.file_type, self.directory))
            return files[-1]

        for entry in os.listdir(self.directory):
            if len(file_name) < len(entry) and file_name == entry[:len(file_name)]:
                return entry

        if not file_name.endswith(self.file_type):
            return file_name + "." + self.file_type

        return file_name


class AtlasWriteFile(AtlasFile):
    def __init__(self, input_name, input_dir, compress, file_type, default_dir):

        super(AtlasWriteFile, self).__init__(
            input_name, input_dir, file_type, default_dir,
        )

        if os.path.exists(self.full_path):
            index = len(self.file_name) - 1
            while index >= 0 and self.file_name[index] in number_characters:
                index -= 1

            if index == len(self.file_name) - 1 or self.file_name[
                index] not in whitespace_characters:  # no end numbers found
                self.file_name += " 1"
            else:
                file_num = int(self.file_name[index + 1:]) + 1
                self.file_name = self.file_name[:index + 1] + str(file_num)

        self.data_file = None
        self._is_open = False
        self.compress = compress

    def open(self):
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        self.data_file = open(self.full_path, "w+")
        self._is_open = True

    def is_open(self):
        return self._is_open

    def write(self, s):
        self.contents += s
        if len(self.contents) > 0x10000:
            self._dump_all()

    def _compress(self):
        """
        Compress the written text file into a gzip.
        :return: None
        """
        with open(self.full_path, "rb") as file:
            self.raw_contents = file.read()

        compressed_data = gzip.compress(self.raw_contents)
        with open(self.full_path, "wb") as file:
            file.write(compressed_data)

    def _dump_all(self):
        """
        Write all data in self.contents. This minimizes OS system calls.
        :return: None
        """
        self.data_file.write(self.contents)
        self.contents = ""

    def close(self):
        pass

    def _close(self):
        """
        If the file hasn't been closed, close and compress it.
        :return: None
        """

        if self.is_open():
            self.close()
            self._dump_all()
            self.data_file.close()
            self.compress()
            self._is_open = False


class AtlasReadFile(AtlasFile):
    def __init__(self, input_name, input_dir, decompress, file_type, default_dir):
        super(AtlasReadFile, self).__init__(
            input_name, input_dir, file_type, default_dir,
        )

        self.open(decompress)

    def open(self, decompress):
        with open(self.full_path, "rb" if decompress else "r") as data_file:
            self.raw_contents = data_file.read()
            self.contents = self.raw_contents

        if decompress:
            self._decompress()

    def _decompress(self):
        self.contents = gzip.decompress(self.raw_contents).decode('utf-8').split("\n")

    def __len__(self):
        return len(self.contents)
