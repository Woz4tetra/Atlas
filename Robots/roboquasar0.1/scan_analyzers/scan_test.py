import math

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.robotplot import RobotPlot


def load_data(datadir, dataset):
    filename = '%s/%s.dat' % (datadir, dataset)
    print('Loading data from %s...' % filename)

    fd = open(filename, 'rt')

    timestamps = []
    scans = []
    odometries = []

    while True:

        s = fd.readline()

        if len(s) == 0:
            break

        toks = s.split()[0:-1]  # ignore ''

        timestamp = int(toks[0])

        odometry = timestamp, int(toks[2]), int(toks[3])

        lidar = [int(tok) for tok in toks[24:]]

        timestamps.append(timestamp)
        scans.append(lidar)
        odometries.append(odometry)

    fd.close()

    return timestamps, scans, odometries


scans = load_data(".", "exp1")

scan_plot = RobotPlot("scan", x_range=(-100, 100), y_range=(-100, 100), marker=".", linestyle='')
plotter = LivePlotter(1, scan_plot)

timestamps, scans, odometries = scans

for scan in scans:
    xs = []
    ys = []

    for index, distance in enumerate(scan):
        angle = index / len(scan) * 2 * math.pi

        xs.append(distance * math.cos(angle))
        ys.append(distance * math.sin(angle))
    scan_plot.update(xs, ys)

    if not plotter.plot():
        break

plotter.close()