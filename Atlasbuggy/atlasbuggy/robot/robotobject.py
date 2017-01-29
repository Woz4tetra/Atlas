"""
The RobotObject class acts a container for data received from and sent to the corresponding robot port.
Data is directed using a RobotInterface class. Every robot object has a unique whoiam ID which is
also defined on the microcontroller.
"""

from multiprocessing import Queue


class RobotObject:
    def __init__(self, whoiam, enabled=True):
        """
        A container for data received from the corresponding microcontroller.

        Make sure the whoiam ID corresponds to the one defined on the microcontroller
        (see templates for details).

        Define object variables here

        :param whoiam: a unique string ID containing ascii characters
        """
        self.whoiam = whoiam
        self.enabled = enabled

        self.command_packets = Queue(maxsize=255)

    def receive_first(self, packet):
        """
        Override this method when subclassing RobotObject if you're expecting initial data

        Initialize any data defined in __init__ here.
        If the whoiam packet contains data, it's passed here. Otherwise, this method isn't called

        :param packet: The first packet received by the robot object's port
        :return: False if program need to exit for some reason, None or True otherwise
        """
        raise NotImplementedError("Please override this method when subclassing RobotObject")

    def receive(self, timestamp, packet):
        """
        Override this method when subclassing RobotObject

        Parse incoming packets received by the corresponding port.
        This method is called on the RobotSerialPort's thread.
        I would recommend ONLY parsing packets here and not doing anything else.
        Use did_update for any event based function calls in the main thread.

        :param timestamp: The time the packet arrived
        :param packet: A packet (string) received from the robot object's port
        :return: False if program need to exit for some reason, None or True otherwise
        """
        raise NotImplementedError("Please override this method when subclassing RobotObject")

    def send(self, packet):
        """
        Do NOT override this method when subclassing RobotObject

        Queue a new packet for sending. The packet end (\n) will automatically be appended

        :param packet: A packet (string) to send to the microcontroller without the packet end character
        :return: None
        """
        self.command_packets.put(packet)
