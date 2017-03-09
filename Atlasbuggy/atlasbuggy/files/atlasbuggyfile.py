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
    """

    """
    def __init__(self, input_name, input_dir, file_type, default_dir):
        """
        :param input_name: name to search for
            can be part of the name. If the desired file is named
                "15;19;32.gzip", input_name can be "15;19"
        :param input_dir: searches in working directory. (must be exact name of the folder)
            If None, the most recently created folder will be used
        :param file_type: file extension (without ".")
        :param default_dir: default directory to search in
        """
        self.input_name = input_name
        self.input_dir = input_dir
        self.file_type = file_type.replace(".", "")
        self.default_dir = default_dir

        self.directory = self.get_abs_dir(self.input_dir)
        self.file_name = self.get_file_name(input_name, self.directory)
        self.full_path = os.path.join(self.directory, self.file_name)

        self.contents = ""
        self.raw_contents = ""

    def get_abs_dir(self, directory):
        """
        Convert input directory to an absolute direcory
        :param directory: self.input_dir
        :return: absolute directory
        """
        abs_default_dir = os.path.abspath(self.default_dir)

        if directory is None:  # if None, search default and grab the last entry
            directories = os.listdir(abs_default_dir)
            # if no directories found, use the default directory
            if len(directories) == 0:
                # raise NotADirectoryError("No directories found in '%s'" % self.default_dir)
                return abs_default_dir

            # make listed dirs absolute
            entries = [os.path.join(abs_default_dir, entry) for entry in directories]

            # sort by date created
            directories = sorted(filter(os.path.isdir, entries), key=lambda x: os.path.getmtime(x))

            # use the most recent folder
            return directories[-1]
        elif directory[0] != "/":
            return os.path.join(abs_default_dir, directory)
        elif directory[0] == "/" and os.path.exists(directory):
            return directory
        else:
            raise NotADirectoryError("Input directory not found! '%s'" % self.input_dir)

    def get_file_name(self, file_name, directory):
        """
        Interpret input file name
        :param file_name: input file name. If None, the most recently created file will be used
        :param directory: absolute directory
        :return: intrepreted file name
        """
        name_with_ext = file_name + "." + self.file_type
        if file_name is None:  # retrieve last file
            entries = os.listdir(self.directory)
            if len(entries) == 0:
                raise FileNotFoundError(
                    "File of type '%s' not found in directory: %s" % (self.file_type, self.directory))

            # make listed files absolute paths
            entries = [os.path.join(self.directory, entry) for entry in entries]

            # remove files that don't have the matching extension
            files = filter(lambda entry: entry.endswith(self.file_type), entries)

            # sort by date created
            files = sorted(files, key=lambda x: os.path.getmtime(x))

            # use the last file
            return files[-1]
        else:
            for entry in os.listdir(self.directory):
                # if there is a perfect match, use it
                if name_with_ext == entry:
                    return entry

                # if the file partially matches, use that entry
                if len(file_name) < len(entry) and entry.endswith(self.file_type) and file_name == entry[:len(file_name)]:
                    file_name = entry

            if not file_name.endswith("." + self.file_type):
                file_name += "." + self.file_type

            return file_name


class AtlasWriteFile(AtlasFile):
    def __init__(self, input_name, input_dir, compress, file_type, default_dir):
        """
        :param input_name: name to search for
            can be part of the name. If the desired file is named
                "15;19;32.gzip", input_name can be "15;19"
        :param input_dir: searches in working directory. (must be exact name of the folder)
            If None, the most recently created folder will be used
        :param compress: compress the file. File type will automatically become gzip
        :param file_type: expected file extension
        :param default_dir: default directory to search in
        """
        if compress:
            file_type = "gzip"

        super(AtlasWriteFile, self).__init__(
            input_name, input_dir, file_type, default_dir,
        )

        # make directories if they don't exit
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

        # if the file name overlaps, append an increasing counter
        if os.path.exists(self.full_path):
            index = len(self.file_name) - 1  # index of file number counter
            while index >= 0 and self.file_name[index] in number_characters:
                index -= 1

            # no end numbers found
            if index == -1 or self.file_name[index] not in whitespace_characters:
                self.file_name += " 1"
            else:
                file_num = int(self.file_name[index + 1:]) + 1
                self.file_name = self.file_name[:index + 1] + str(file_num)

        # reference of write error
        self.data_file = None
        self._is_open = False
        self.compress = compress

    def open(self):
        self.data_file = open(self.full_path, "w+")
        self._is_open = True

    def is_open(self):
        return self._is_open

    def write(self, s):
        """
        Append s to current file contents.
        Every 65536 characters, the current contents will be written (saves memory)
        :param s: string
        """
        self.contents += s
        if len(self.contents) > 0x10000:
            self._dump_all()

    def _compress(self):
        """
        Compress the written text file into a gzip.
        """
        if self.compress:
            with open(self.full_path, "rb") as file:
                self.raw_contents = file.read()

            compressed_data = gzip.compress(self.raw_contents)
            with open(self.full_path, "wb") as file:
                file.write(compressed_data)

    def _dump_all(self):
        """
        Write all data in self.contents. This minimizes OS system calls.
        """
        self.data_file.write(self.contents)
        self.contents = ""

    def close(self):
        """
        If the file hasn't been closed, close and compress it (if compress is True).
        """

        if self.is_open():
            self._dump_all()
            self.data_file.close()
            self._compress()
            self._is_open = False


class AtlasReadFile(AtlasFile):
    def __init__(self, input_name, input_dir, decompress, file_type, default_dir):
        """
        File is opened when AtlasReadFile is initialized

        :param input_name: name to search for
            can be part of the name. If the desired file is named
                "15;19;32.gzip", input_name can be "15;19"
        :param input_dir: searches in working directory. (must be exact name of the folder)
            If None, the most recently created folder will be used
        :param decompress: decompress the file. File type will automatically become gzip
        :param file_type: expected file extension
        :param default_dir: default directory to search in
        """
        if decompress:
            file_type = "gzip"

        super(AtlasReadFile, self).__init__(
            input_name, input_dir, file_type, default_dir,
        )

        self._open(decompress)

    def _open(self, decompress):
        """
        Open the file and put it in self.contents
        :param decompress: decompress the file (gzip)
        """
        with open(self.full_path, "rb" if decompress else "r") as data_file:
            self.raw_contents = data_file.read()
            self.contents = self.raw_contents

        if decompress:
            self._decompress()

    def _decompress(self):
        self.contents = gzip.decompress(self.raw_contents).decode('utf-8')

    def __len__(self):
        return len(self.contents)
