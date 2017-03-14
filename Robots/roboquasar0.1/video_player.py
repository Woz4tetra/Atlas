from roboquasar import RoboQuasar, file_sets, map_sets
from atlasbuggy.files.atlasbuggyfile import AtlasFile
from atlasbuggy.files.logfile import Parser

file_name, directory = file_sets["data day 10"][0]
parser = Parser(file_name, directory)

checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets["cut 3"]

robot = RoboQuasar(False, checkpoint_map_name, inner_map_name, outer_map_name, map_dir)
robot.logitech.show = True
robot.is_live = False

robot.open_cameras(parser.file_name_no_ext, parser.input_dir)

robot.pipeline.start()

while True:
    if input("> ") == 'q':
        robot.pipeline.close()
        break
