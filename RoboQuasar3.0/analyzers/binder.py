"""
Written by Ben Warwick

data.py, written for RoboQuasar3.0
Version 3/10/2015
=========

Finds goal position based on a supplied current position
"""

class Binder:
    def __init__(self, map_name, directory=None):
        self.map = Map(map_name, directory)
        # set to None so that it will be able to start at any point on the track
        self.prev_bind = None

    def bind(self, position):
        if (self.prev_bind is None or self.prev_bind >= (len(self.map.data) - 1)
                or self.prev_bind < 0):
            # finds the smallest distance between the point and the map
            self.prev_bind = self.find_nearest(position)
            return self.map.data[self.prev_bind + 1]

        for index in range(self.prev_bind, len(self.map.data)):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map.data[index + 1]

        for index in range(self.prev_bind):
            if self.is_near(index, position):
                self.prev_bind = index
                return self.map.data[index + 1]

        self.prev_bind = self.find_nearest(position)
        return self.map.data[self.prev_bind + 1]

    def find_nearest(self, position):
        map_dist = [0] * len(self.map.data)
        for index in range(len(map_dist)):
            dlat = abs(float(self.map.data[index][0] - position[0]))
            dlong = abs(float(self.map.data[index][1] - position[1]))
            dist = ((dlat ** 2) + (dlong ** 2)) ** 0.5
            map_dist[index] = dist
        smallest_value = min(map_dist)
        index = map_dist.index(smallest_value)
        return index

    def is_near(self, index, position):
        dx = abs(float(self.map.data[index][0]) - position[0])
        dy = abs(float(self.map.data[index][1]) - position[1])
        dist = ((dx ** 2) + (dy ** 2)) ** 0.5

        if index + 2 < len(self.map.data):
            acc_dlat = abs(float(self.map.data[index][0] - self.map.data[index + 1][0]))
            acc_dlong = abs(float(self.map.data[index][1] - self.map.data[index + 1][1]))

        else:
            acc_dlat = abs(float(self.map.data[index][0] - self.map.data[index - 1][0]))
            acc_dlong = abs(float(self.map.data[index][1] - self.map.data[index - 1][1]))

        accuracy = ((acc_dlat ** 2 + acc_dlong ** 2) ** 0.5) / 2

        return dist <= accuracy


def binder_test():
    binder = Binder("Mon Mar  7 17;54;59 2016 GPS Map.csv", 0,
        26.611000061, 56.40650177)
    for row in binder.map.data:
        print("%f,%f" % (row[0], row[1]))

    pos1 = (26.6, 56.329)
    pos2 = (26.594, 56.34)
    pos3 = (26.59, 56.35)
    pos4 = (26.584, 56.357)
    pos5 = (26.582, 56.36)

    pos = binder.bind(pos1)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos2)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos3)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos4)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

    pos = binder.bind(pos5)
    pre_bind = binder.prev_bind
    print((binder.map.data[pre_bind]), "map[pre_bind]")
    print((pos, pre_bind), "pos, pre_bind")

def test_shift():
    test_obj = Map("2015-11-08 07_58_30.csv", 0,
                   40.4405106744, -79.9425712322)
    for row in test_obj.data:
        print("%f,%f" % (row[0], row[1]))
