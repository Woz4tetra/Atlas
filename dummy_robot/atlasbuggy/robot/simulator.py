from atlasbuggy.logfiles.parser import Parser


class RobotInterfaceSimulator:
    def __init__(self, file_name, directory, start_index=0, end_index=-1, *robot_objects):
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object
        self.parser = Parser(file_name, directory, start_index, end_index)
        self.current_index = 0

        self.prev_percent = 0
        self.percent = 0

    def print_percent(self):
        percent = 100 * self.parser.content_index / len(self.parser.contents)
        self.percent = int(percent * 10)
        if self.percent != self.prev_percent:
            self.prev_percent = self.percent
            print(("%0.1f" % percent) + "%", end='\r')

    def packet_received(self, timestamp, whoiam, packet):
        return True

    def run(self):
        for index, timestamp, whoiam, packet in self.parser:
            if whoiam in self.objects.keys():
                if timestamp == -1:
                    self.objects[whoiam].receive_first(packet)
                    continue
                else:
                    self.objects[whoiam].receive(packet)

            self.current_index = index
            if not self.packet_received(timestamp, whoiam, packet):
                print("packet_received signalled to exit")
                break

        self.close()

    def close(self):
        pass
