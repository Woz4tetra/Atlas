from atlasbuggy.logs.parser import Parser


class RobotInterfaceSimulator:
    def __init__(self, file_name, directory, start_index=0, end_index=-1, *robot_objects):
        self.objects = {}
        for robot_object in robot_objects:
            self.objects[robot_object.whoiam] = robot_object
        self.parser = Parser(file_name, directory, start_index, end_index)
        self.current_index = 0

    def packet_received(self, timestamp, whoiam):
        return True

    def run(self):
        for index, timestamp, whoiam, packet in self.parser:
            if timestamp == -1:
                self.objects[whoiam].receive_first(packet)
            else:
                self.objects[whoiam].receive(packet)
                if not self.packet_received(timestamp, whoiam):
                    print("packet_received signalled to exit")
                    break

            self.current_index = index

        self.close()

    def close(self):
        pass
