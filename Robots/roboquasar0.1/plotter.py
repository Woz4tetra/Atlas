from threading import Thread

from atlasbuggy.interface.simulated import RobotSimulator

from roboquasar import RoboQuasar, file_sets, map_sets  # , video_sets

# file_name, directory = file_sets["rolls day 3"][0]
file_name, directory = file_sets["data day 10"][0]
# file_name = "15;21"
# directory = "data_days/2017_Mar_13"
checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

# def thing():
#     while True:
#         status = input("> ")
#         if status == "q":
#             simulator.exit()
#
#
# t = Thread(target=thing)
# t.daemon = True
# t.start()

robot = RoboQuasar(True, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
simulator = RobotSimulator(file_name, directory, robot, debug_enabled=False)

simulator.run()
