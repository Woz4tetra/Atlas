
from buggypi.microcontroller.logger import get_map

class Waypoints:
    def __init__(self, map_name, left_angle_limit, right_angle_limit):
        self.map = get_map(map_name)
        self.left_limit = left_angle_limit
        self.right_limit = right_angle_limit

    def get_goal(self, state):
        x = state["x"]
        y = state["y"]


