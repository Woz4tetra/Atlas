from atlasbuggy.interface.simulated import RobotSimulator
from roboquasar import RoboQuasar, file_sets, map_sets  # , video_sets

# file_name, directory = file_sets["rolls day 3"][0]
file_name, directory = file_sets["rolls day 5"][-1]
# file_name = "22;02"
# directory = "data_days/2017_Mar_21"
checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["buggy"]

# def thing():
#     while True:
#         status = input("> ")
#         if status == "q":
#             simulator.exit()
#
# t = Thread(target=thing)
# t.daemon = True
# t.start()

robot = RoboQuasar(True, checkpoint_map_name, inner_map_name, outer_map_name, map_dir, enable_kalman=True,
                   enable_cameras=True, use_log_file_maps=True, day_mode=True)
robot.manual_mode = False
simulator = RobotSimulator(file_name, directory, robot, debug_enabled=False)

simulator.run()
