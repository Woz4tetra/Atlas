"""
Possible errors robot ports, objects and interface might face.
"""


class RobotPortBaseError(Exception):
    def __init__(self, error_message, prev_packet_info, port=None):
        if port is not None:
            full_messages = []
            with port.message_lock:
                while not port.error_message.empty():
                    full_messages.append(port.error_message.get())

            port_error_info = ""
            for full_message in full_messages:
                if type(port.whoiam) == str:
                    whoiam_info = '%s' % port.whoiam
                else:
                    whoiam_info = str(port.whoiam)

                if len(full_message) > 0:
                    port_error_info += "\nError message from port:\n" \
                                       "%s\n" % full_message

                port_error_info += "\nAddress: '%s', ID: %s" % (
                    port.address, whoiam_info
                )
                if any(prev_packet_info):  # if any values evaluate to True, print info
                    port_error_info += "\n\nPrevious packet sent (ID: %s, time: %s):\n%s" % (
                        prev_packet_info[0], prev_packet_info[1], repr(prev_packet_info[2]))
        else:
            port_error_info = ""
        super(RobotPortBaseError, self).__init__(error_message + port_error_info)


class ReceivePacketError(RobotPortBaseError):
    """Failed to parse a packet from serial"""


class LoopSignalledError(Exception):
    """Loop method threw an exception"""


class StartSignalledError(Exception):
    """User's start method threw an exception"""


class CloseSignalledExitError(Exception):
    """Loop method threw an exception"""


class PacketReceivedError(Exception):
    """packet_received method threw an exception"""


class RobotObjectReceiveError(Exception):
    """robot_object.receive method threw an exception"""

    def __init__(self, whoiam, packet):
        super(RobotObjectReceiveError, self).__init__("Input packet from '%s': %s" % (whoiam, repr(packet)))


class RobotObjectInitializationError(Exception):
    """Object passed isn't a RobotObject"""


class RobotSerialPortUnassignedError(RobotPortBaseError):
    """Port was open successfully but no objects use it"""


class RobotSerialPortWhoiamIdTaken(RobotPortBaseError):
    """whoiam ID is already being used by another port"""


class RobotSerialPortNotConfiguredError(RobotPortBaseError):
    """Port was not opened successfully"""


class RobotObjectNotFoundError(Exception):
    """Failed to assign a robot object to a port"""


class RobotSerialPortClosedPrematurelyError(RobotPortBaseError):
    """serial object signalled it wasn't open"""


class RobotSerialPortReadPacketError(RobotPortBaseError):
    """A port failed to call read_packets successfully"""


class RobotSerialPortWritePacketError(RobotPortBaseError):
    """A port failed to call write_packets successfully"""


class RobotSerialPortSignalledExitError(RobotPortBaseError):
    """Port signalled to exit"""


class RobotSerialPortFailedToStopError(RobotPortBaseError):
    """Port didn't stop"""
