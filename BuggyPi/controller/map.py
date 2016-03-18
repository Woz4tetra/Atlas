# handles map reading and finds goal position based on a supplied current position

import time
import csv

def get_map(directory=None):
    with open(directory, 'rb') as csvfile:
        map_reader = csv.reader(csvfile, delimiter=',', quotechar='|')
        parsed = [[float(row[0]), float(row[1])] for row in map_reader]
        return parsed


def write_map(data, directory=None):
    if directory == None:
        directory = time.strftime("%c").replace(":", ";")
    with open(directory + ".csv", 'wb') as csvfile:
        map_writer = csv.writer(csvfile, delimiter=',',
                                quotechar='|',
                                quoting=csv.QUOTE_MINIMAL)
        for row in data:
            assert len(row) == 2
            map_writer.writerow(row)


class Binder:
    def __init__(self, map, accuracy=0.000002):
        self.map = map
        self.accuracy = accuracy
        self.prevBind = 0

    def bind(self, position):
        if self.prevBind >= len(self.map) or self.prevBind < 0:
            raise Exception("bind outside track")

        for index in range(self.prevBind, len(self.map)):
            if self.is_near(index, position):
                self.prevBind = index
                return index + 1

        for index in range(len(self.map)):
            if self.is_near(index, position):
                self.prevBind = index
                return index + 1

        return False

    def is_near(self, index, position):
        dist_lat = abs(float(self.map[index][0]) - position[0])
        dist_lng = abs(float(self.map[index][1]) - position[1])
        dist = ((dist_lat ** 2) + (dist_lng ** 2) ** (0.5))

        if dist > self.accuracy:
            return False

        elif dist <= self.accuracy:
            return True


if __name__ == '__main__':
    import sys

    sys.path.insert(0, '../')

    import config

    map = get_map(config.get_dir(":maps") + "2015-11-08 07_58_30.csv")
    binder = Binder(map)

    pos = binder.bind((40.44051147069823, -79.94248582057649))
    pre_bind = binder.prevBind
    print((binder.map[pre_bind]))
    print((pos, pre_bind))
