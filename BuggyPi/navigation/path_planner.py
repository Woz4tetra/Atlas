"""
class Node:
    def __init__(self, contents, parent=None, *children):
        self.parent = parent
        self.children = children
        self.contents = contents

class Graph:
    def __init__(self, initial_state):
        self.root = Node(initial_state)

    def graph_from_dict(self, graph_dict):

class RRT:
    def __init__(self, initial_state, tree_size, dt, map):
        self.map = map
        self.initial_state = initial_state
        self.tree_size = tree_size
        self.dt = dt

        self.graph = Graph(initial_state)


    def generate_rrt(self):
        for counter in range(self.tree_size):
            state_random = self.random_state()
            state_nearest = self.nearest_neighbor(state_random)


    def random_state(self):
        pass

    def nearest_neighbor(self, state):
"""
