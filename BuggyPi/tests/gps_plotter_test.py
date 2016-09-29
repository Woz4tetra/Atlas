import sys

sys.path.insert(0, "../")

from microcontroller.logger import Parser, get_points
from analyzers.plotter import plot_gps, plot_gps_checkpoints, plot_all_gps

print("hi")


parser = Parser("Jun 22 2016/Wed Jun 22 19;44;34 2016.txt",None)
checkpoints = get_points()

lat_data, long_data = [], []  # all gps data
check_lat, check_long = [], []  # checkpoint gps points
check_lat_data, check_long_data = [], []  # gps points when checkpoint was recorded

for data in parser:
        if data[1] == 'gps':
            lat_data.append(data[2]['lat'])
            long_data.append(data[2]['long'])
        elif data[1] == 'checkpoint':
            lat, long = checkpoints[data[2]['num']]
            check_lat.append(lat)
            check_long.append(long)

            lat, long = float(data[2]['lat']), float(data[2]['long'])
            check_lat_data.append(lat)
            check_long_data.append(long)

plot_gps_checkpoints("Jun 22 2016/Wed Jun 22 19;44;34 2016.txt",None)

print(len(lat_data), len(check_lat))

