from roboquasar import RoboQuasar, file_sets, map_sets
from atlasbuggy.files.logfile import Parser, Logger


def run(record, play):
    assert record != play

    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

    robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
    robot.logitech.show = True

    file_source = None

    if play:
        file_name, directory = file_sets["data day 10"][0]
        file_source = Parser(file_name, directory)
        robot.is_live = False

    if record:
        file_source = Logger(None, ("data_days", None))
        robot.is_live = True

    assert file_source is not None

    robot.open_cameras(file_source.file_name_no_ext, file_source.input_dir)

    robot.pipeline.start()

    while True:
        if input("> ") == 'q':
            robot.logitech.close()
            robot.pipeline.close()
            break

run(record=False, play=True)
