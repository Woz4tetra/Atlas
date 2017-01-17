"""
Possible errors robot ports, objects and interface might face.
"""


class RobotObjectBaseError(Exception):
    def __init__(self, error_message, port=None):
        if port is not None:
            port_error_message = ""
            if port.error_message is not None:
                for line in port.error_message:
                    port_error_message += str(line).strip() + "\n"

            if type(port.whoiam) == str:
                whoiam_info = '%s' % port.whoiam
            else:
                whoiam_info = str(port.whoiam)
            port_error_info = "\nError message from port:\n%s\naddress: '%s', ID: %s" % (
                port_error_message, port.address, whoiam_info
            )
        else:
            port_error_info = ""
        super(RobotObjectBaseError, self).__init__(error_message + port_error_info)


class ReceivePacketError(RobotObjectBaseError):
    """Failed to parse a packet from serial"""


class LoopSignalledError(Exception):
    """Loop method threw an exception"""


class CloseSignalledExitError(Exception):
    """Loop method threw an exception"""


class PacketReceivedError(Exception):
    """packet_received method threw an exception"""


class RobotObjectReceiveError(Exception):
    """robot_object.receive method threw an exception"""

    def __init__(self, whoiam, packet):
        super(RobotObjectReceiveError, self).__init__("Input packet from '%s': %s" % (whoiam, repr(packet)))


class RobotSerialPortUnassignedError(RobotObjectBaseError):
    """Port was open successfully but no objects use it"""


class RobotSerialPortWhoiamIdTaken(RobotObjectBaseError):
    """whoiam ID is already being used by another port"""



class RobotSerialPortNotConfiguredError(RobotObjectBaseError):
    """Port was not opened successfully"""


class RobotObjectNotFoundError(Exception):
    """Failed to assign a robot object to a port"""


class RobotSerialPortClosedPrematurelyError(Exception):
    """serial object signalled it wasn't open"""


class RobotSerialPortReadPacketError(Exception):
    """A port failed to call read_packets successfully"""


class RobotSerialPortWritePacketError(Exception):
    """A port failed to call write_packets successfully"""


class RobotSerialPortSignalledExitError(Exception):
    """Port signalled to exit"""


class RobotSerialPortTimeoutError(Exception):
    """Port thread timed out"""
