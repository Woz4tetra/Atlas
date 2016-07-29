import math
import numpy as np

import pygame
from pygame.locals import *

from buggypi.microcontroller.logger import *
from navigation.buggypi_filter import BuggyPiFilter
from navigation.waypoint_picker import Waypoints
from standard_params import standard_params
from collections import defaultdict


class Plotter:
    def __init__(self, file_name, directory, map_name, map_dir=None, width=800, height=800,**plot_options):
        self.robot_params = defaultdict(lambda: False, **standard_params)

        self.map_name = map_name
        self.map_dir = map_dir
        self.plot_options = plot_options

        self.checkpoints = get_map("checkpoints.txt")
        self.waypoints = Waypoints(
            self.map_name, 1, map_dir
        )

        initial_long, initial_lat = self.checkpoints[0]
        second_long, second_lat = self.checkpoints[1]

        bearing = BuggyPiFilter.get_gps_bearing(
            initial_long, initial_lat, second_long, second_lat
        )
        bearing = (-bearing - math.pi / 2) % (2 * math.pi)
        self.filter = BuggyPiFilter(
            self.robot_params['counts_per_rotation'], self.robot_params['wheel_radius'],
            self.robot_params['front_back_dist'], self.robot_params['max_speed'],
            self.robot_params['left_angle_limit'], self.robot_params['right_angle_limit'],
            self.robot_params['left_servo_limit'], self.robot_params['right_servo_limit'],
            initial_long, initial_lat, bearing
        )
        print(initial_long, initial_lat, bearing)

        self.parser = Parser(file_name, directory)

        self.prev_imu_time = 0.0
        self.prev_enc_time = 0.0

        self.dt_enc = None
        self.dt_imu = None

        self.heading_arrow_len = 0.00002
        self.arrow_color = 'orange'

        self.bearing_arrow_len = 0.000005

        # ----- data to record -----

        self.state = [self.filter.initial_long, self.filter.initial_lat]
        # self.state_heading = [(self.filter.initial_long - 0.0001,
        #                        self.filter.initial_long - 0.0001 +
        #                        self.heading_arrow_len),
        #                       (self.filter.initial_lat + 0.0001,
        #                        self.filter.initial_lat + 0.0001),
        #                       'darkgreen']

        self.gps_data = []  # all gps data
        self.check_lat, self.check_long = [], []  # checkpoint gps points

        self.state_x_gps = [self.filter.initial_long]  # put red dots on the state line where the GPS updated
        self.state_y_gps = [self.filter.initial_lat]

        self.bearing_data = []

        self.recorded_state = []

        self.recorded_heading_counter = 0
        self.recorded_heading_freq = 10

        self.heading_counter = 0
        self.heading_freq = 50

        self.waypoint_lines = []
        self.waypoint_color = 'burlywood'
        self.waypoint_counter = 0
        self.waypoint_freq = 75

        self.boardwidth = width
        self.boardheight = height

        self.surface = pygame.display.set_mode((self.boardwidth,self.boardheight))



    def record_waypoints(self, state):
        if self.waypoint_counter % self.waypoint_freq == 0:
            goal_x, goal_y = self.waypoints.get_goal(state)
            #     self.waypoint_lines.append((state["x"], goal_x))
            #     self.waypoint_lines.append((state["y"], goal_y))
            #     self.waypoint_lines.append(self.waypoint_color)
            #     self.ax.arrow(state["x"], state["y"], goal_x, goal_y, head_width=0.0000001, head_length=0.0000001, fc='k', ec='k')
            self.ax.annotate("",
                             xy=(goal_x, goal_y), xycoords='data',
                             xytext=(state["x"], state["y"]), textcoords='data',
                             arrowprops=dict(arrowstyle="->",
                                             connectionstyle="arc",
                                             linewidth=1),
                             )

        self.waypoint_counter += 1


    def step(self, index, timestamp, name, values):
        if name == "gps":
            if self.plot_options["plot_calculated_state"]:
                state = self.filter.update_gps(
                    timestamp, values["long"], values["lat"])

                self.state.append([state["x"], state["y"]])

                if self.plot_options["plot_state_gps_dots"]:
                    self.state_x_gps.append(state["x"])
                    self.state_y_gps.append(state["y"])

            if self.plot_options["plot_gps"]:
                self.gps_data.append([values["long"],values["lat"]])

        elif name == "imu":
            if self.plot_options["plot_calculated_state"]:
                state = self.filter.update_imu(timestamp, values["yaw"])
                self.state.append([state["x"], state["y"]])

                # if self.plot_options["waypoints"]:
                    # self.record_waypoints(state)
        elif name == "encoder":
            if self.plot_options["plot_calculated_state"]:
                state = self.filter.update_encoder(timestamp, values["counts"])
                self.state.append([state["x"], state["y"]])
                # if self.plot_options["waypoints"]:
                #     self.record_waypoints(state)

        elif name == "servo":
            if self.plot_options["plot_calculated_state"]:
                self.filter.update_servo(values)
        elif name == "motors":
            if self.plot_options["plot_calculated_state"]:
                self.filter.update_motors(values)
        elif name == "state":
            if self.plot_options["plot_recorded_state"]:
                self.recorded_state.append([values["x"], values["y"]])
                # if self.plot_options["waypoints"]:
                #     self.record_waypoints(values)

        percent = 100 * index / len(self.parser)
        print(("%0.4f" % percent) + "%", end='\r')

    def static_plot(self):
        pygame.draw.rect(self.surface, pygame.Color('white'), (0,0, self.boardwidth,self.boardheight), 0)
        
        for index, timestamp, name, values in self.parser:
            self.step(index, timestamp, name, values)

        print("plotting...")
        if self.plot_options["plot_map"]:
            # gps_map = np.array(self.waypoints.map)
            # pygame.draw.lines(surface, color, closed, pointlist, width) 
            pygame.draw.lines(self.surface,pygame.Color('purple'), False, self.waypoints.map)

        if self.plot_options["plot_gps"]:
            pygame.draw.lines(self.surface,pygame.Color('red'), False, self.gps_data)
            # plt.plot(*self.bearing_data)

        if self.plot_options["plot_checkpoints"]:
            # checkpoints_map = np.array(self.checkpoints)
            for x,y in self.checkpoints:
                pygame.draw.circle(self.surface, pygame.Color('blue'), (x,y), .005, 1)

        if self.plot_options["plot_calculated_state"]:
            if self.plot_options["plot_state_gps_dots"]:
                for index in range(len(self.state_x_gps)):
                    pygame.draw.circle(self.surface, pygame.Color('red'), (self.state_x_gps[index], self.state_y_gps[index]), 0.005, 1)
            pygame.draw.lines(self.surface,pygame.Color('green'), False, self.state)

        if self.plot_options["plot_recorded_state"]:
            pygame.draw.lines(self.surface,pygame.Color('orange'), False, self.recorded_state)


        pygame.display.update()

        for event in pygame.event.get():
                if event.type == QUIT:
                    pygame.quit()
                    sys.exit()
        # if self.plot_options["waypoints"]:
        #     plt.plot(*self.waypoint_lines)


    def write_maps(self, skip=1, map_source_x=None, map_source_y=None):
        print("You are about to create map based on the current plot. "
              "Continue? (y/n)", end="")

        proceed = None
        while proceed != "" and proceed != "y" and proceed != "n":
            proceed = input(": ").lower()

        if proceed == "y":
            if map_source_x is None:
                map_source_x = self.state_x
            if map_source_y is None:
                map_source_y = self.state_y
            assert len(map_source_x) == len(map_source_y)

            map_dir = project.get_dir(":maps") + self.parser.file_name[:-4]
            gpx_map_dir = project.get_dir(":gpx") + self.parser.file_name[:-4]

            map_file = open("%s map.txt" % map_dir, "w+")
            map_gpx_file = open("%s gpx map.gpx" % gpx_map_dir, "w+")

            assert len(map_source_x) == len(map_source_y)
            map_file.write("long, lat\n")
            for index in range(0, len(map_source_x), skip):
                map_file.write("%s, %s\n" % (
                    map_source_x[index], map_source_y[index]))

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
                    min(map_source_y), min(map_source_x), max(map_source_y),
                    max(map_source_x)
                ))
            map_gpx_file.write("<trk><name>checkpoints</name><trkseg>\n")
            for index in range(0, len(map_source_x), skip):
                map_gpx_file.write('<trkpt lat="%s" lon="%s"></trkpt>\n' % (
                    map_source_y[index], map_source_x[index]))

            map_gpx_file.write('</trkseg></trk></gpx>\n')

            map_file.close()
            map_gpx_file.close()

            print("Map created successfully!", map_dir)
            print("GPX map created successfully!", gpx_map_dir)


if len(sys.argv) == 2:
    file_name = sys.argv[1]
    directory = None
    map_name = -1
    map_dir = ":gpx"
elif len(sys.argv) == 3:
    file_name, directory = sys.argv[1:]
    map_name = -1
    map_dir = ":gpx"
elif len(sys.argv) == 4:
    file_name, directory, map_name = sys.argv[1:]
    map_dir = ":gpx"
elif len(sys.argv) == 5:
    file_name, directory, map_name, map_dir = sys.argv[1:]

else:
    file_name = 7
    directory = 'Jul 22 2016'
    map_name = "test goal track.gpx"
    map_dir = ":gpx"

try:
    file_name = int(file_name)
except ValueError:
    pass

plotter = Plotter(
    file_name, directory, map_name, map_dir,
    plot_map=True, plot_gps=True, plot_checkpoints=True,
    plot_calculated_state=True, plot_recorded_state=False, waypoints=True,
    plot_state_gps_dots=False
)
plotter.static_plot()
# plotter.write_maps(300)#, plotter.long_data, plotter.lat_data)
