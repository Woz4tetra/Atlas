
class Waypoints:
    def __init__(self, gps_map, left_limit, right_limit):
        self.map = gps_map
        self.left_limit = left_limit
        self.right_limit = right_limit

    def get_goal(self, state):
        x = state["x"]
        y = state["y"]


