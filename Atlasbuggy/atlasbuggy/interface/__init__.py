"""
This module handles all interfacing with serial ports and passing data from them.
The idea is every microcontroller gets one corresponding robot object and port (which runs
on its own process).
What data you send over serial is up to you. There is no strict packet protocol.
"""

from threading import Thread

from atlasbuggy.interface.simulated import RobotSimulator
from atlasbuggy.interface.live import RobotRunner

robot_interface = None


def simulate(file_name, directory, robot, start_index=0, end_index=-1, debug_enabled=False, extra_func=None):
    """
    Launch a simulation using the provided Robot object. The function will exit when the simulator has finished
    Run on a separate thread if you don't want this behavior (see the close function)

    :param file_name: log file name
    :param directory: log directory name
    :param robot: a subclass instance of the atlasbuggy.robot.Robot class
    :param start_index: line number to start simulating from
    :param end_index: line number to end simulation
    :param debug_enabled: print debug messages in the log and from the simulator
    :return: instance of RobotSimulator class
    """
    global robot_interface
    robot_interface = RobotSimulator(file_name, directory, robot, start_index, end_index, debug_enabled)
    robot_interface.run(extra_func)

    return robot_interface


def run(robot, joystick=None, log_data=True, log_name=None, log_dir=None, debug_prints=False, blocking=True, extra_func=None):
    """
    Launch a live robot using the provided Robot object. The function will exit when the runner has finished
    Run on a separate thread if you don't want this behavior (see the close function)

    :param robot: a subclass instance of the atlasbuggy.robot.Robot class
    :param joystick: a subclass instance of the atlasbuggy.buggyjoystick.BuggyJoystick class
    :param log_data: enable data logging (debug prints logged even if they are not enabled)
    :param log_name: log file name (today's date is substituted if None (YYYY_MM_DD))
    :param log_dir: log directory (today's date is substituted if None (YYYY_MM_DD))
        Tuple feature: ("some_data", None) -> produces "some_data/YYYY_MM_DD"
    :param debug_prints: print debug messages
    :return: instance of RobotRunner class
    """
    global robot_interface
    robot_interface = RobotRunner(robot, joystick, log_data, log_name, log_dir, debug_prints)
    robot_interface.run(extra_func)

    return robot_interface


# def close():
#     """
#     Close the current robot interface (simulator or live). Use if run or simulated is called on a separate thread
#     example:
#
#     from threading import Thread
#     def run_robot():
#         run(robot)
#
#     t = Thread(target=run_robot)
#     t.daemon = True
#     t.start()
#
#     ...
#
#     close()
#     """
#     global robot_interface
#     if robot_interface is not None:
#         robot_interface.exit()
