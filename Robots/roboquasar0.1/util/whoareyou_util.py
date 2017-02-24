from threading import Thread
import serial.tools.list_ports

from atlasbuggy.robot.port import RobotSerialPort

if len(serial.tools.list_ports.comports()) == 0:
    print("No ports found!")

message = ""


def discover_port(port_info):
    global message
    if port_info.serial_number is not None:
        robot_port = RobotSerialPort(port_info, True, None, None, None, None)
        if not robot_port.configured:
            # robot_port.stop()
            # raise RobotSerialPortNotConfiguredError("Port not configured!", robot_port)
            robot_port.stop()
            # pprint.pprint(port_info.__dict__)
            # print("-----------------------------------")
            # print("address '%s' does not abide atlasbuggy protocol!" % (port_info.device))
            # print("-----------------------------------")
            message += "address '%s' does not abide atlasbuggy protocol!\n" % (port_info.device)
        else:
            message += "address '%s' has ID '%s'\n" % (
                port_info.device, robot_port.whoiam if robot_port.whoiam is not None else '')
            robot_port.stop()


threads = []
for port_info in serial.tools.list_ports.comports():
    thread = Thread(target=discover_port, args=(port_info,))
    threads.append(thread)
    thread.start()

for thread in threads:
    thread.join()

print()
print(message)