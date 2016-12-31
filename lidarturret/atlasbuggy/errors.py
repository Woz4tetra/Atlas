# ----- interface.py exceptions -----

class RobotObjectBaseError(Exception):
    def __init__(self, error_message, port):
        port_error_message = ""
        for line in port.error_message:
            port_error_message += str(line).strip() + "\n"

        port_error_info = "\n\naddress: '%s', ID: '%s'\n\nError message from port:\n%s" % (
            port.address, port.whoiam, port_error_message
        )
        super(RobotObjectBaseError, self).__init__(error_message + port_error_info)


class ReceivePacketError(RobotObjectBaseError):
    """Failed to parse a packet from serial"""


class RobotSerialPortUnassignedError(RobotObjectBaseError):
    """Port was open successfully but no objects use it"""


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


# ----- buggyjoystick.py exceptions -----


class JoysticksNotFoundError(Exception):
    """No joysticks found"""
