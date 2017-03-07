from atlasbuggy.interface.simulated import RobotSimulator
from atlasbuggy.interface.live import RobotRunner


def simulate(file_name, directory, robot, start_index=0, end_index=-1, debug_enabled=False):
    RobotSimulator(file_name, directory, robot, start_index, end_index, debug_enabled).run()


def run(robot, joystick=None, log_data=True, log_name=None, log_dir=None, debug_prints=False):
    RobotRunner(robot, joystick, log_data, log_name, log_dir, debug_prints).run()
