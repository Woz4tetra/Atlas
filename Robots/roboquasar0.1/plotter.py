from atlasbuggy.interface import simulate

from roboquasar import RoboQuasar, file_sets, map_sets


# file_name, directory = file_sets["rolls day 3"][0]
file_name, directory = file_sets["data day 9"][4]

robot = RoboQuasar(False, True, True, *map_sets["cut"])
simulate(file_name, directory, robot, debug_enabled=True)
