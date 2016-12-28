from atlasbuggy.microcontroller.logger import *
import math


def run_modify(default_file, default_dir):
    file_name, directory = parse_arguments(default_file, default_dir)
    parser = Parser(file_name, directory)

    response = ""
    while response != 'y' and response != 'n':
        response = input("Proceed? (y/n): ")

    new_file_name = "modified " + parser.file_name_no_ext
    new_dir = parser.local_dir
    logger = Logger(new_file_name, new_dir)

    if response == 'y':
        for index, timestamp, name, values in parser:
            new_values = modify_line(index, timestamp, name, values)
            if new_values is not None:
                logger.enq(name, new_values, timestamp)
        logger.record()
        logger.close()

        print("Finished! Modified the log file and put it in \n"
              "'%s' in directory '%s'" % (new_file_name, new_dir))
        print("Make sure you delete the pickled file as well!")


def modify_line(index, timestamp, name, values):
    if name == "imu":
        values["gx"] /= 2 * math.pi
        values["gy"] /= 2 * math.pi
        values["gz"] /= 2 * math.pi
    if name == "gps":
        if values["long"] == 0.0:
            return None
        if values["lat"] == 0.0:
            return None
    return values


run_modify("22;09;22, Sat Nov 12 2016.txt", "Nov 12 2016")
