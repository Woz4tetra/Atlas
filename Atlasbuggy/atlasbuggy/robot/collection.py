from multiprocessing import Queue


class RobotObjectCollection:
    def __init__(self, *whoiam_ids, enabled=True):
        self.whoiam_ids = whoiam_ids
        self.enabled = enabled

        self.is_live = True

        self.command_packets = {}
        for whoiam in self.whoiam_ids:
            self.command_packets[whoiam] = Queue(maxsize=255)

    def receive_first(self, whoiam, packet):
        pass

    def receive(self, timestamp, whoiam, packet):
        pass

    def send(self, whoiam, packet):
        if self.is_live and whoiam in self.whoiam_ids:
            self.command_packets[whoiam].put(packet)