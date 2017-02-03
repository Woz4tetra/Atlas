"""
This module handles all features related to log files including creating and parsing them.
"""

import time

# logs file data separators (end markers)
time_whoiam_sep = ":"  # timestamp
whoiam_packet_sep = ";"  # data name

# all data before this date may not work correctly with the current code
obsolete_data = "Jan 14 2017"  # "Nov 14 2016"
log_file_type = "gzip"
# pickle_file_type = "pkl"

log_directory = ":logs"
# pickle_directory = ":pickled"
# pickled_sim_directory = ":simulations"

log_folder_format = '%b %d %Y'
log_file_format = '%H;%M;%S, %a %b %d %Y'

packet_types = {
    "object" : "<",       # from a robot object
    "user"   : "|",       # user logged
    "command": ">",       # command sent
    "error"  : "<!!!!>",  # printed error message

    "<"      : "object",
    "|"      : "user",
    ">"      : "command",
    "<!!!!>" : "error",
}


def todays_log_folder():
    """Generate a logs folder name based on the current date"""
    return time.strftime(log_folder_format) + "/"


def filename_now():
    return time.strftime(log_file_format)
