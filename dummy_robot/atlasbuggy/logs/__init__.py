import time

# log file data separators (end markers)
time_whoiam_sep = ":\t"  # timestamp
whoiam_packet_sep = ";\t"  # data name

# all data before this date may not work correctly with the current code
obsolete_data = "Nov 14 2016"
log_file_type = "txt"  # easily change file types (not that you should need to)
pickle_file_type = "pkl"

log_directory = ":logs"
pickle_directory = ":pickled"
pickled_sim_directory = ":simulations"

log_folder_format = '%b %d %Y'
log_file_format = '%H;%M;%S, %a %b %d %Y'


def todays_log_folder():
    """Generate a log folder name based on the current date"""
    return time.strftime(log_folder_format) + "/"


def filename_now():
    return time.strftime(log_file_format)