import sys

sys.path.insert(0, "../")

from microcontroller.logger import *

for (dir_path, dir_names, file_names) in os.walk(project.get_dir(":logs")):
    for file_name in file_names:
        if file_name[-4:] == ".txt":
            parser = Parser(file_name, dir_path)
            logger = Logger(file_name, dir_path)

            for index, timestamp, name, values in parser:
                if name == "imu":
                    values["yaw"] *= -1
                logger.enq(name, values, timestamp)
            logger.record()
