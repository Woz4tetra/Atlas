import math
import sys

from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from robots.robot import Robot
from microcontroller.logger import *
import project


class PlotterBot(Robot):
    def __init__(self, file_name, directory,
                 initial_long=None, initial_lat=None, initial_heading=0.0):
        super(PlotterBot, self).__init__(
            initial_long, initial_lat, initial_heading
        )
        self.parser = Parser(file_name, directory)

        self.prev_imu_time = 0.0
        self.prev_enc_time = 0.0

        self.dt_enc = None
        self.dt_imu = None

        self.heading_arrow_len = 0.00002
        self.arrow_color = 'orange'

        self.bearing_arrow_len = 0.000005

        # ----- data to record -----

        self.state_x = [self.initial_long]
        self.state_y = [self.initial_lat]
        # self.state_heading = [(self.initial_long - 0.0001,
        #                        self.initial_long - 0.0001 +
        #                        self.heading_arrow_len),
        #                       (self.initial_lat + 0.00001,
        #                        self.initial_lat + 0.00001),
        #                       'darkgreen']
        self.state_heading = []

        self.lat_data, self.long_data = [], []  # all gps data
        self.check_lat, self.check_long = [], []  # checkpoint gps points

        self.state_x_gps = [self.initial_long]
        self.state_y_gps = [self.initial_lat]

        self.bearing_data = []

        self.recorded_state_x = []
        self.recorded_state_y = []
        self.recorded_state_heading = []

        self.recorded_heading_counter = 0
        self.recording_heading_freq = 30

        self.heading_counter = 0
        self.heading_freq = 50

        # ----- initialize figures -----

        self.fig = plt.figure(0)
        ax = self.fig.gca()
        ax.plot(self.initial_long, self.initial_lat, 'o', color='black',
                markersize=10)
        self.fig.canvas.set_window_title(
            self.parser.local_dir + self.parser.file_name[:-4])

    def step(self, index, timestamp, name, values):
        if name == "gps":
            state = self.pi_filter.update_gps(
                timestamp, values["long"], values["lat"])

            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

            self.state_x_gps.append(state["x"])
            self.state_y_gps.append(state["y"])

            self.long_data.append(values["long"])
            self.lat_data.append(values["lat"])

            x0 = values["long"]
            y0 = values["lat"]
            x1 = x0 + self.bearing_arrow_len * math.cos(
                self.pi_filter.gps_bearing)
            y1 = y0 + self.bearing_arrow_len * math.sin(
                self.pi_filter.gps_bearing)
            self.bearing_data.append((x0, x1))
            self.bearing_data.append((y0, y1))
            self.bearing_data.append('orange')

        elif name == "imu":
            state = self.pi_filter.update_imu(timestamp, values["yaw"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)
        elif name == "encoder":
            state = self.pi_filter.update_encoder(timestamp, values["counts"])
            self.heading_counter = \
                self.record_state_data(state, self.state_x, self.state_y,
                                       self.state_heading, self.heading_counter,
                                       self.arrow_color, self.heading_freq)

        elif name == "servo":
            self.pi_filter.update_servo(values)
        elif name == "motors":
            self.pi_filter.update_motors(values)
        elif name == "checkpoint":
            long, lat = self.checkpoints[values['num']]
            self.check_long.append(long)
            self.check_lat.append(lat)
        elif name == "state":
            self.recorded_heading_counter = \
                self.record_state_data(values, self.recorded_state_x,
                                       self.recorded_state_y,
                                       self.recorded_state_heading,
                                       self.recorded_heading_counter,
                                       'purple', self.recording_heading_freq)

        percent = 100 * index / len(self.parser)
        print(str(int(percent)) + "%", end='\r')

    def record_state_data(self, state, state_x_data, state_y_data,
                          state_heading_data, heading_counter, heading_color,
                          frequency):
        state_x_data.append(state["x"])
        state_y_data.append(state["y"])

        if heading_counter % frequency == 0 and \
                        len(state_x_data) > 0 and len(state_y_data) > 0:
            x0 = state_x_data[-1]
            y0 = state_y_data[-1]
            speed = (state["vx"] ** 2 + state["vy"] ** 2) ** 0.5
            speed = speed * 0.75 + 0.75
            percent_speed = abs(
                self.pi_filter.max_speed - speed) / self.pi_filter.max_speed
            x1 = x0 + self.heading_arrow_len * percent_speed * math.cos(
                state["angle"])
            y1 = y0 + self.heading_arrow_len * percent_speed * math.sin(
                state["angle"])
            state_heading_data.append((x0, x1))
            state_heading_data.append((y0, y1))
            state_heading_data.append(heading_color)

        return heading_counter + 1

    def static_plot(self, plot_recorded_state, plot_calculated_state):
        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)

        print("plotting...")
        plt.plot(self.long_data, self.lat_data, "r", label="GPS")
        plt.plot(self.check_long, self.check_lat, "o", label="Checkpoints")
        plt.plot(*self.bearing_data)

        if plot_calculated_state:
            plt.plot(self.state_x_gps, self.state_y_gps, "ro",
                     label="GPS updated",
                     markersize=4)
            plt.plot(self.state_x, self.state_y, 'g', label="state xy")
            plt.plot(*self.state_heading)
        if plot_recorded_state:
            plt.plot(self.recorded_state_x, self.recorded_state_y, 'g',
                     label="state xy")
            plt.plot(*self.recorded_state_heading)

        plt.legend(loc='upper right', shadow=True, fontsize='x-small')

        plt.show()

    def write_maps(self, skip):
        print("You are about to create map based on the current plot. "
              "Continue? (y/n)", end="")

        proceed = None
        while proceed != "" and proceed != "y" and proceed != "n":
            proceed = input(": ").lower()

        if proceed != "n":
            map_file = open("%s%s map.txt" % (project.get_dir(":maps"),
                                              self.parser.file_name[:-4]), "w+")
            map_gpx_file = open("%s%s gpx map.gpx" %
                                (project.get_dir(":gpx"),
                                 self.parser.file_name[:-4]), "w+")

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
