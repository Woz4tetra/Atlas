from atlasbuggy.interface import simulate

from roboquasar import RoboQuasar, file_sets, map_sets


# file_name, directory = file_sets["rolls day 3"][0]
file_name, directory = file_sets["data day 8"][4]

robot = RoboQuasar(True, *map_sets["cut 3"])
simulate(file_name, directory, robot, debug_enabled=True)
