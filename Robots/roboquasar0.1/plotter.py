from atlasbuggy.interface.simulated import RobotSimulator
from roboquasar import RoboQuasar, file_sets, map_sets  # , video_sets

file_name, directory = file_sets["push practice 4"][-1]
# file_name, directory = file_sets["rolls day 10"][-1]
# file_name = "22;02"
# directory = "data_days/2017_Mar_21"

# def thing():
#     while True:
#         status = input("> ")
#         if status == "q":
#             simulator.exit()
#
# t = Thread(target=thing)
# t.daemon = True
# t.start()

robot = RoboQuasar(True, "buggy 2", enable_cameras=True, use_log_file_maps=False, show_cameras=True)
robot.manual_mode = False
# robot.bozo_filter.init_compass("200")
simulator = RobotSimulator(file_name, directory, robot, debug_enabled=False)

simulator.run()
