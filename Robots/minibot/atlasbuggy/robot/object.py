"""
The RobotObject class acts a container for data received from and sent to the corresponding robot port.
Data is directed using a RobotRunner class. Every robot object has a unique whoiam ID which is
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
        :param enabled: disable or enable object
        """
        self.whoiam = whoiam
        self.enabled = enabled
        self.baud = None  # set this externally AFTER super().__init__ is called if a different baud rate is desired
        self.is_live = None
        self.command_packets = Queue(maxsize=255)

    def receive_first(self, packet):
        """
        Override this method when subclassing RobotObject if you're expecting initial data

        Initialize any data defined in __init__ here.
        If the initialization packet is not an empty string, it's passed here. Otherwise, this method isn't called

        :param packet: The first packet received by the robot object's port
        :return: a string if the program needs to exit ("done" or "error"), None if everything is ok
        """
        raise NotImplementedError("Please override this method when subclassing RobotObject")

    def receive(self, timestamp, packet):
        """
        Override this method when subclassing RobotObject

        Parse incoming packets received by the corresponding port.
        I would recommend ONLY parsing packets here and not doing anything else.

        :param timestamp: The time the packet arrived
        :param packet: A packet (string) received from the robot object's port
        :return: a string if the program needs to exit ("done" or "error"), None if everything is ok
        """
        raise NotImplementedError("Please override this method when subclassing RobotObject")

    def send(self, packet):
        """
        Do NOT override this method when subclassing RobotObject

        Queue a new packet for sending. The packet end (\n) will automatically be appended

        :param packet: A packet (string) to send to the microcontroller without the packet end character
        """
        if self.enabled and self.is_live:
            self.command_packets.put(packet)

    def __str__(self):
        return "%s(whoiam=%s)\n\t" % (self.__class__.__name__, self.whoiam)
