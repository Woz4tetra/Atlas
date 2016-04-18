import sys
import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from analyzers.logger import Recorder, get_data
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder


def main(log_data=False):
    if log_data:
        log = Recorder(directory="Test Day 6")

    timestamps, data, length = get_data(
        "Test Day 5/Sun Apr 10 18;30;08 2016.csv",
        ["gps_long", "gps_lat", "encoder"],
        density=100)
    gps_long, gps_lat, encoder = data

    converter = Converter(gps_long[0], gps_lat[0], 0.000003)  # long, lat

    heading_filter = HeadingFilter()
    position_filter = PositionFilter()

    binder = Binder("Track Field Map.csv", gps_long[0], gps_lat[0])
    bind_x, bind_y = 0, 0


    x, y = [], []
    for index in range(len(binder.map.data)):
        x.append(binder.map.data[index][0])
        y.append(binder.map.data[index][1])
    plt.plot(x, y)

    x, y = [], []
    lines = []
    for index in range(1, length):
        gps_heading, bind_heading = converter.convert_heading(
            gps_long[index], gps_lat[index], bind_x, bind_y)
        gps_x, gps_y, enc_dist = \
            converter.convert_position(
                gps_long[index], gps_lat[index], encoder[index])

        bind_x, bind_y = binder.bind((gps_x, gps_y))
        hypotenuse = 10  # 0.5 * ((dx ** 2 + dy ** 2) ** .5)

        lines.append((gps_x, gps_x + hypotenuse * np.cos(gps_heading)))
        lines.append((gps_y, gps_y + hypotenuse * np.sin(gps_heading)))
        lines.append('r')

        x.append(gps_x)
        y.append(gps_y)

    plt.plot(*lines)
    plt.plot(x, y)

    plt.show()


if __name__ == '__main__':
    print(__doc__)
    main()
