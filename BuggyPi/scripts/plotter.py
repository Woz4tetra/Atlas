import sys

sys.path.insert(0, "../")

from robots.plotterbot import PlotterBot

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
        # file_name = 'Thu Jun 23 20;53;43 2016.txt'
        # directory = 'Jun 23 2016'
    try:
        file_name = int(file_name)
    except ValueError:
        pass

    # plot_everything(file_name, directory)
    plot = PlotterBot(file_name, directory)
    plot.static_plot(plot_recorded_state=True, plot_calculated_state=True)
    # plot.live_plot()
    plot.write_maps(100)
