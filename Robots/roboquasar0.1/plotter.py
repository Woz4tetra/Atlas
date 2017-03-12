from threading import Thread

from atlasbuggy.interface.simulated import RobotSimulator

from roboquasar import RoboQuasar, file_sets, map_sets  # , video_sets

# file_name, directory = file_sets["rolls day 3"][0]
file_name, directory = file_sets["data day 8"][4]
checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]


# video_name, video_dir = video_sets[""][0]

def thing():
    while True:
        status = input("> ")
        if status == "q":
            simulator.exit()


t = Thread(target=thing)
t.daemon = True
t.start()

robot = RoboQuasar(True, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
simulator = RobotSimulator(file_name, directory, robot, debug_enabled=False)

robot.playback(simulator.file_name_no_ext, simulator.directory)

simulator.run()