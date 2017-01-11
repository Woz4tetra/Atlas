from multiprocessing import Queue


class RobotObject:
    def __init__(self, whoiam):
        """
        A container for data received from the corresponding microcontroller.

        Make sure the whoiam ID corresponds to the one defined on the microcontroller
        (see templates for details).

        Define object variables here

        :param whoiam:
        """
        self.whoiam = whoiam

        self.command_packets = Queue(maxsize=255)

    def receive_first(self, packet):
        """
        Overwrite this method when subclassing RobotObject if you're expecting initial data

        Initialize any data defined in __init__ here.
        If the who_am_i packet contains data, it's passed here. Otherwise, this method isn't called

        :param packet: The first packet received by the robot object's port
        :return: None
        """
        pass

    def receive(self, packet):
        """
        Overwrite this method when subclassing RobotObject

        Parse incoming packets received by the corresponding port.
        This method is called on the RobotSerialPort's thread.
        I would recommend ONLY parsing packets here and not doing anything else.
        Use did_update for any event based function calls in the main thread.

        :param packet: A packet (string) received from the robot object's port
        :return: None
        """
        pass

    def send(self, packet):
        """
        Do NOT overwrite this method when subclassing RobotObject

        Queue a new packet for sending. The packet end (\n) will automatically be appended

        :param packet: A packet (string) to send to the microcontroller without the packet end character
        :return: None
        """
        self.command_packets.put(packet)