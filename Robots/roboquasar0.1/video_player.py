import os
import threading
from roboquasar import RoboQuasar, map_sets, video_sets, file_sets
from atlasbuggy.files.atlasbuggyfile import AtlasFile


def avi_to_mp4():
    def convert(path, new_path):
        os.system(
            "avconv -i %s -r 32 -c:v libx264 -c:a copy %s.mp4" % (
                path, new_path
            )
        )

    directory = "videos/rolls/2017_Mar_25_raw"
    output_dir = "videos/rolls/2017_Mar_25"
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    path_threads = []
    for entry in os.listdir(directory):
        if entry.endswith("avi"):
            path = os.path.join(directory, entry)
            new_path = os.path.join(output_dir, entry)
            ext_index = new_path.rfind(".")
            new_path = new_path[:ext_index]
            print("Converting", entry)
            path_threads.append(threading.Thread(target=convert, args=(path, new_path)))
            path_threads[-1].start()
            if len(path_threads) > 8:
                for thread in path_threads:
                    thread.join()
                path_threads = []

    for thread in path_threads:
        thread.join()


def run(record, play):
    assert record != play

    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

    robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir, pipeline=2)
    robot.left_camera.show = True

    file_name = None
    directory = None
    file_format = "mp4"

    if play:
        # file_name, directory = file_sets["push practice 2"][6]
        # file_finder = AtlasFile(file_name, directory, "gzip", "logs", False, False)
        # file_name = file_finder.file_name_no_ext.replace(";", "_")

        file_name, directory, file_format = video_sets["data day 12"][0]
        robot.is_live = False

    if record:
        file_name, directory = AtlasFile.format_path_as_time(None, ("data_days", None), "%H_%M_%S", "%Y_%b_%d")
        robot.is_live = True

    assert file_name is not None and directory is not None

    robot.open_cameras(file_name, directory, file_format)

    # robot.pipeline.read_thread.start()

    try:
        robot.left_pipeline.run()
    except KeyboardInterrupt:
        pass

    print("Closing cameras")
    robot.left_camera.close()
    robot.left_pipeline.close()

# avi_to_mp4()
run(record=False, play=True)
