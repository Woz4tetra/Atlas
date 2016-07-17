import sys

sys.path.insert(0, "../")

from robots.plotterbot import PlotterBot
from robots.standard_config import *

if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name = sys.argv[1]
        directory = sys.argv[2]
    else:
        file_name = 2
        directory = "Jul 11 2016"
        # file_name = ":random"
        # directory = ":random"
        # file_name = 'Mon Jul 11 19;50;34 2016.txt'
        # directory = 'Jul 11 2016'
    try:
        file_name = int(file_name)
    except ValueError:
        pass

    bearing = get_gps_bearing(
        -71.420864, 42.427317, -71.420795, 42.427332
    )
    bearing = (-bearing - math.pi / 2) % (2 * math.pi) + 0.1
    update_properties(initial_heading=bearing)

    plot = PlotterBot(file_name, directory, properties)

    plot.static_plot(plot_recorded_state=True, plot_calculated_state=True)
    # plot.write_maps(100)
