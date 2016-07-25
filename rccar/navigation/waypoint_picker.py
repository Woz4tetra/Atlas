from buggypi.microcontroller.logger import get_map


class Waypoints:
    def __init__(self, map_name, offset=1, map_dir=None):
        self.map = get_map(map_name, map_dir)
        self.offset = offset

    def get_goal(self, state):
        x0 = state["x"]
        y0 = state["y"]

        smallest_dist = None
        goal_index = 0
        for index in range(len(self.map)):
            x1, y1 = self.map[index]
            dist = ((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0)) ** 0.5
            if smallest_dist is None or dist < smallest_dist:
                smallest_dist = dist
                goal_index = index
        goal_index = (goal_index + self.offset) % len(self.map)
        # print("%i, %0.6f, %0.6f" % (goal_index, x0, y0))
        # print("%0.6f, %0.6f" % (self.map[goal_index][0], self.map[goal_index][1]))
        return self.map[goal_index]
