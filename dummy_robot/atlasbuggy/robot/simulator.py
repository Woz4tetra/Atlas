"""
RobotInterfaceSimulator imitates RobotInterface except its data source is a log file.
"""

from atlasbuggy.logfiles.parser import Parser
from atlasbuggy.logfiles import packet_types


class RobotInterfaceSimulator:
    def __init__(self, file_name, directory, start_index=0, end_index=-1, *robot_objects):
        """
        :param file_name: log file name or number
        :param directory: directory to search in
        :param start_index:
        :param end_index:
        :param robot_objects:
        """
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object
        self.parser = Parser(file_name, directory, start_index, end_index)
        self.current_index = 0

        self.ids_used = set()
        self.ids_received = set()
        self.prev_whoiam = None

        self.prev_percent = 0
        self.percent = 0

        self.dt = None

    def print_percent(self):
        percent = 100 * self.parser.index / len(self.parser.contents)
        self.percent = int(percent * 10)
        if self.percent != self.prev_percent:
            self.prev_percent = self.percent
            print(("%0.1f" % percent) + "%", end='\r')

    def object_packet(self, timestamp):
        pass

    def user_packet(self, timestamp, packet):
        pass

    def command_packet(self, timestamp, packet):
        pass

    def did_receive(self, whoiam):
        self.ids_used.add(whoiam)
        return whoiam == self.prev_whoiam

    def run(self):
        for index, packet_type, timestamp, whoiam, packet in self.parser:
            self.dt = timestamp
            self.prev_whoiam = whoiam
            self.ids_received.add(whoiam)

            if whoiam in self.objects.keys():
                if timestamp == -1:
                    self.objects[whoiam].receive_first(packet)
                    continue
                else:
                    self.objects[whoiam].receive(timestamp, packet)

            self.current_index = index

            if packet_type == "object":
                self.object_packet(timestamp)
            elif packet_type == "user":
                self.user_packet(timestamp, packet)
            elif packet_type == "command":
                self.command_packet(timestamp, packet)

        if self.ids_received != self.ids_used:
            if len(self.ids_received - self.ids_used) > 0:
                print("Warning IDs unused:")
                for id in self.ids_received - self.ids_used:
                    print("\t", id)
            if len(self.ids_used - self.ids_received) > 0:
                print("Warning IDs not in log file:")
                for id in self.ids_used - self.ids_received:
                    print("\t", id)

        self.close()

    def close(self):
        pass
