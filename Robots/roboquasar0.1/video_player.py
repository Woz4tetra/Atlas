from roboquasar import RoboQuasar, map_sets
from atlasbuggy.files.atlasbuggyfile import AtlasFile


def run(record, play):
    assert record != play

    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

    robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
    # robot.left_camera.show = True

    file_name = None
    directory = None

    if play:
        # file_name, directory = file_sets["data day 10"][0]
        file_name, directory = "18_33_06", "data_days/2017_Mar_14"
        # file_name, directory = "18_36_30", "data_days/2017_Mar_14"
        robot.is_live = False
    if record:
        file_name, directory = AtlasFile.format_path_as_time(None, ("data_days", None), "%H_%M_%S", "%Y_%b_%d")
        robot.is_live = True

    assert file_name is not None and directory is not None

    robot.open_cameras(file_name, directory)

    try:
        robot.pipeline.run()
    except KeyboardInterrupt:
        pass

    print("Closing cameras")
    robot.left_camera.close()
    robot.pipeline.close()


run(record=True, play=False)
