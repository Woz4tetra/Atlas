import serial.tools.list_ports
import pprint

from atlasbuggy.robot.robotport import RobotSerialPort

if len(serial.tools.list_ports.comports()) == 0:
    print("No ports found!")

for port_info in serial.tools.list_ports.comports():
    if port_info.serial_number is not None:
        pprint.pprint(port_info.__dict__)
        robot_port = RobotSerialPort(port_info, True, None, None, None, None)
        if not robot_port.configured:
            # robot_port.stop()
            # raise RobotSerialPortNotConfiguredError("Port not configured!", robot_port)
            robot_port.close_port()
            print("-----------------------------------")
            print("address '%s' does not abide atlasbuggy protocol!" % (port_info.device))
            print("-----------------------------------")
        else:
            print("-----------------------------------")
            print("address '%s' has ID '%s'" % (port_info.device, robot_port.whoiam if robot_port.whoiam is not None else ''))
            print("-----------------------------------")
            robot_port.stop()
