import serial.tools.list_ports
import pprint

from atlasbuggy.robot.robotport import RobotSerialPort
from atlasbuggy.robot.errors import RobotSerialPortNotConfiguredError

for port_info in serial.tools.list_ports.comports():
    if port_info.serial_number is not None:
        pprint.pprint(port_info.__dict__)
        robot_port = RobotSerialPort(port_info, True, None, None, None, None)
        if not robot_port.configured:
            robot_port.stop()
            raise RobotSerialPortNotConfiguredError("Port not configured!", robot_port)

        print("-----------------------------------")
        print("address '%s' has ID '%s'" % (port_info.device, robot_port.whoiam if robot_port.whoiam is not None else ''))
        print("-----------------------------------")
        robot_port.stop()
