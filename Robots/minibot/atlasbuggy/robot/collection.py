"""
The RobotObjectCollection class acts a container for data received from and sent to multiple robot ports.
Data is directed using a RobotRunner class. Every robot collection has unique whoiam ID's which are
also defined on the microcontrollers.

This class should be used if multiple microcontrollers act as one sensor
"""

from multiprocessing import Queue


class RobotObjectCollection:
    def __init__(self, *whoiam_ids, enabled=True):
        """
        A container for data received from the corresponding microcontrollers.

        Make sure the whoiam ID's corresponds to the one defined on the microcontrollers
        (see templates for details).

        Define object variables here

        :param whoiam_ids: unique string ID's containing ascii characters
        :param enabled: disable or enable object
        """
        self.whoiam_ids = whoiam_ids
        self.enabled = enabled
        self.baud = None
        self.is_live = None
        self.command_packets = {}

        for whoiam in self.whoiam_ids:
            self.command_packets[whoiam] = Queue(maxsize=255)

    def receive_first(self, whoiam, packet):
        """
        Override this method when subclassing RobotObjectCollection if you're expecting initial data

        Initialize any data defined in __init__ here.
        If the initialization packet is not an empty string, it's passed here. Otherwise, this method isn't called

        :param whoiam: The destination object's whoiam ID
        :param packet: The first packet received by the robot object's port
        :return: a string if the program needs to exit ("done" or "error"), None if everything is ok
        """
        raise NotImplementedError("Please override this method when subclassing RobotObjectCollection")

    def receive(self, timestamp, whoiam, packet):
        """
        Override this method when subclassing RobotObjectCollection

        Parse incoming packets received by the corresponding port.
        I would recommend ONLY parsing packets here and not doing anything else.

        :param timestamp: The time the packet arrived
        :param whoiam: The destination object's whoiam ID
        :param packet: A packet (string) received from the robot object's port
        :return: a string if the program needs to exit ("done" or "error"), None if everything is ok
        """
        raise NotImplementedError("Please override this method when subclassing RobotObjectCollection")

    def send(self, whoiam, packet):
        """
        Do NOT override this method when subclassing RobotObjectCollection

        Queue a new packet for sending. The packet end (\n) will automatically be appended

        :param whoiam: whoiam ID to send packet to
        :param packet: A packet (string) to send to the microcontroller without the packet end character
        """
        if self.is_live and whoiam in self.whoiam_ids:
            self.command_packets[whoiam].put(packet)

    def __str__(self):
        return "%s(whoiam's=%s)\n\t" % (self.__class__.__name__, self.whoiam_ids)