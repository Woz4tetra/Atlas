from atlasbuggy.interface.simulated import RobotSimulator
from roboquasar import RoboQuasar, file_sets, map_sets  # , video_sets

# file_name, directory = file_sets["push practice 4"][-1]
file_name, directory = file_sets["rolls day 11"][-1]
# file_name, directory = file_sets["rolls day 10"][-1]

robot = RoboQuasar(
    True, "buggy 2",
    enable_cameras=False, use_log_file_maps=True, show_cameras=True, animate=True, enable_pid=True
)
robot.manual_mode = False
simulator = RobotSimulator(
    file_name, directory, robot, debug_enabled=False,
    # start_index=19000
    start_index=35000
)

simulator.run()
