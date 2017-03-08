from atlasbuggy.interface.simulated import RobotSimulator
from atlasbuggy.interface.live import RobotRunner


robot_interface = None


def simulate(file_name, directory, robot, start_index=0, end_index=-1, debug_enabled=False):
    global robot_interface
    robot_interface = RobotSimulator(file_name, directory, robot, start_index, end_index, debug_enabled)
    robot_interface.run()


def run(robot, joystick=None, log_data=True, log_name=None, log_dir=None, debug_prints=False):
    global robot_interface
    robot_interface = RobotRunner(robot, joystick, log_data, log_name, log_dir, debug_prints)
    robot_interface.run()


def close():
    global robot_interface
    if robot_interface is not None:
        robot_interface.exit()
