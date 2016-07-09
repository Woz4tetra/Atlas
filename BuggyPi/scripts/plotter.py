import math
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from microcontroller.logger import *
from navigation.buggypi_filter import BuggyPiFilter


class FilterPlotter:
    def __init__(self, file_name, directory=None):
        # ----- initialize filter -----

        self.parser = Parser(file_name, directory)
        # self.initial_gps = self.parser.get("gps", 2)[2]
        # self.initial_long = self.initial_gps["long"]
        # self.initial_lat = self.initial_gps["lat"]

        self.checkpoints = get_checkpoints()
        self.initial_long, self.initial_lat = self.checkpoints[9]

        self.pi_filter = BuggyPiFilter(
            self.initial_long, self.initial_lat, 0.0, 6, 0.097, 0.234, 0.88
        )

        self.prev_imu_time = 0.0
        self.prev_enc_time = 0.0

        self.dt_enc = None
        self.dt_imu = None

        self.heading_arrow_len = 0.00002

        # ----- data to record -----

        self.state_x = [self.initial_long]
        self.state_y = [self.initial_lat]
        self.state_heading = [(self.initial_long - 0.0001,
                               self.initial_long - 0.0001 +
                               self.heading_arrow_len),
                              (self.initial_lat + 0.00001,
                               self.initial_lat + 0.00001),
                              'darkgreen']

        self.lat_data, self.long_data = [], []  # all gps data
        self.check_lat, self.check_long = [], []  # checkpoint gps points

        self.state_x_gps = [self.initial_long]
        self.state_y_gps = [self.initial_lat]

        self.bearing_data = []

        self.heading_counter = 0

        # ----- initialize figures -----

        self.fig = plt.figure(0)
        ax = self.fig.gca()
        ax.plot(self.initial_long, self.initial_lat, 'o', color='black',
                markersize=10)
        self.fig.canvas.set_window_title(
            self.parser.local_dir + self.parser.file_name[:-4])

    def update_state_data(self, state):
        self.state_x.append(state["x"])
        self.state_y.append(state["y"])

        if self.heading_counter % 15 == 0:
            x0 = self.state_x[-1]
            y0 = self.state_y[-1]
            speed = (state["vx"] ** 2 + state["vy"] ** 2) ** 0.5
            speed = speed * 0.75 + 0.75
            percent_speed = abs(
                self.pi_filter.max_speed - speed) / self.pi_filter.max_speed
            x1 = x0 + self.heading_arrow_len * percent_speed * math.cos(
                state["angle"])
            y1 = y0 + self.heading_arrow_len * percent_speed * math.sin(
                state["angle"])
            self.state_heading.append((x0, x1))
            self.state_heading.append((y0, y1))
            self.state_heading.append('orange')

        self.heading_counter += 1

    def step(self, index, timestamp, name, values):
        if name == "gps":
            state = self.pi_filter.update_gps(
                timestamp, values["long"], values["lat"])

            self.heading_counter = 0
            self.update_state_data(state)

            self.state_x_gps.append(state["x"])
            self.state_y_gps.append(state["y"])

            self.long_data.append(values["long"])
            self.lat_data.append(values["lat"])

            x0 = values["long"]
            y0 = values["lat"]
            x1 = x0 + 0.00001 * math.cos(self.pi_filter.gps_bearing)
            y1 = y0 + 0.00001 * math.sin(self.pi_filter.gps_bearing)
            self.bearing_data.append((x0, x1))
            self.bearing_data.append((y0, y1))
            self.bearing_data.append('blue')

        elif name == "imu":
            state = self.pi_filter.update_imu(timestamp, -values["yaw"])
            self.update_state_data(state)

        elif name == "encoder":
            state = self.pi_filter.update_encoder(timestamp, values["counts"])
            self.update_state_data(state)

        elif name == "servo":
            self.pi_filter.update_servo(values[None])
        elif name == "motors":
            self.pi_filter.update_motors(values[None])
        elif name == "checkpoint":
            long, lat = self.checkpoints[values['num']]
            self.check_long.append(long)
            self.check_lat.append(lat)

        percent = 100 * index / len(self.parser)
        if self.almost_equal(percent % 5, 0):
            # doesn't work in pycharm terminal...
            # print(str(int(percent)) + "%", end='\r')

            print(str(int(percent)) + "%")

    @staticmethod
    def almost_equal(num1, num2, epsilon=0.005):
        return abs(num1 - num2) < epsilon

    def static_plot(self):
        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)

        print("plotting...")
        plt.plot(self.long_data, self.lat_data, "r", label="GPS")
        plt.plot(self.check_long, self.check_lat, "o", label="Checkpoints")
        plt.plot(self.state_x_gps, self.state_y_gps, "ro", label="GPS updated",
                 markersize=4)
        plt.plot(self.state_x, self.state_y, 'g', label="state xy")
        plt.plot(*self.state_heading)
        plt.plot(*self.bearing_data)
        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def live_plot(self):
        # TODO: Find a not terrible way to do this (it's really slow)

        plt.ion()

        # fig = plt.figure(0)
        # fig.canvas.set_window_title(
        #     self.parser.local_dir + self.parser.file_name[:-4])
        #
        # plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)
            plt.plot(self.long_data, self.lat_data, "r", label="GPS")
            plt.plot(self.check_long, self.check_lat, "o", label="Checkpoints")
            plt.plot(self.state_x_gps, self.state_y_gps, "ro",
                     label="GPS updated",
                     markersize=4)
            plt.plot(self.state_x, self.state_y, 'g', label="state xy")
            plt.plot(*self.state_heading)

            plt.draw()
            plt.pause(0.0001)

        plt.ioff()
        plt.show()

    def write_maps(self, skip):
        map_file = open(
            "../microcontroller/maps/%s map.txt" % self.parser.file_name[:-4],
            "w+")
        map_gpx_file = open(
            "../microcontroller/maps/gpx maps/%s map.gpx" % self.parser.file_name[
                                                            :-4],
            "w+")

        assert len(self.state_x) == len(self.state_y)
        map_file.write("long, lat\n")
        for index in range(0, len(self.state_x), skip):
            map_file.write("%s, %s\n" % (
                self.state_x[index], self.state_y[index]))

        map_gpx_file.write("""<?xml version="1.0" encoding="ISO-8859-1" standalone="no" ?>\n<metadata>
    <gpx creator="WTracks" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns="http://www.topografix.com/GPX/1/1" version="1.1" xsi:schemaLocation="http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd">
    <metadata>
      <name>checkpoints</name>
      <desc></desc>
      <author><name>WTracks - Online GPX track editor</name></author>
      <link href="https://wtracks.appspot.com/">
        <text>WTracks</text>
        <type>text/html</type>
      </link>
      <time>2016-06-22T19-23-22Z</time>""")
        map_gpx_file.write(
            '<bounds minlat="%s" minlon="%s" maxlat="%s" '
            'maxlon="%s"/></metadata>\n' % (
                min(self.state_y), min(self.state_x), max(self.state_y),
                max(self.state_x)
            ))
        map_gpx_file.write("<trk><name>checkpoints</name><trkseg>\n")
        for index in range(0, len(self.state_x), skip):
            map_gpx_file.write('<trkpt lat="%s" lon="%s"></trkpt>\n' % (
                self.state_y[index], self.state_x[index]))

        map_gpx_file.write('</trkseg></trk></gpx>\n')

        map_file.close()
        map_gpx_file.close()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        file_name = sys.argv[1]
        directory = None
    elif len(sys.argv) == 3:
        file_name = sys.argv[1]
        directory = sys.argv[2]
    else:
        file_name = 0
        directory = "Jun 26 2016"
        # file_name = ":random"
        # directory = ":random"
        # file_name = 'Thu Jun 23 20;53;43 2016.txt'
        # directory = 'Jun 23 2016'
    try:
        file_name = int(file_name)
    except ValueError:
        pass

    # plot_everything(file_name, directory)
    plot = FilterPlotter(file_name, directory)
    # plot.static_plot()
    plot.live_plot()
    # plot.create_map(50)
