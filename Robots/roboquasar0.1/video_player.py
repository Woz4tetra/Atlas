from roboquasar import RoboQuasar, file_sets, map_sets
from atlasbuggy.files.logfile import Parser, Logger
from atlasbuggy.files.atlasbuggyfile import AtlasFile
import time

def run(record, play):
    assert record != play

    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

    robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
    robot.logitech.show = True

    file_name = None
    directory = None

    if play:
        # file_name, directory = file_sets["data day 10"][0]
        file_name, directory = "18_33_06", "data_days/2017_Mar_14"
        # file_name, directory = "18_36_30", "data_days/2017_Mar_14"
        robot.is_live = False

    if record:
        file_name, directory = None, ("data_days", None)
        robot.is_live = True

    assert file_name is not None and directory is not None

    robot.open_cameras(file_name, directory)

    robot.pipeline.run()

    print("Closing cameras")
    robot.logitech.close()
    robot.pipeline.close()

run(record=False, play=True)
