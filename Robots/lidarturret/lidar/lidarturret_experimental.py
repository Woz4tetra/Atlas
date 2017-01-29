import math

from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection
from atlasbuggy.robot.robotcollection import RobotObjectCollection


class DynamicList:
    def __init__(self, *item):
        self.l = list(item)
        self.index = 0
        self.end_index = 0

    def append(self, *x):
        if len(self.l) <= self.index:
            if len(x) > 1:
                self.l.append(list(x))
            else:
                self.l.append(x[0])
        else:
            if len(x) > 1:
                for sub_index in range(len(x)):
                    self.l[self.index][sub_index] = x[sub_index]
            else:
                self.l[self.index] = x[0]
        self.index += 1

    def get(self):
        return self.l[0:self.end_index]

    def cap(self):
        self.end_index = self.index
        self.index = 0

    def __getitem__(self, item):
        return self.l[item % self.end_index]

    def __len__(self):
        return self.end_index


class LidarTurret(RobotObjectCollection):
    def __init__(self, enable_slam=True):
        self.current_tick = 0  # current tick received
        self.ticks_per_rotation = 0
        self.prev_hz_time = 0
        self.update_rate_hz = 0.0

        self.dist_timestamp = 0
        self.distances = DynamicList()

        self.tick_timestamp = 0
        self.ticks = DynamicList()

        self.rotations = 0
        self.prev_time = 0

        self.point_cloud_xs = DynamicList()
        self.point_cloud_ys = DynamicList()

        lidar_range = (-100, 100)
        # lidar_max_range = (-10000, 10000)
        self.point_cloud_plot = RobotPlot(
            "point cloud",
            x_range=lidar_range, y_range=lidar_range, range_offset=1,
            marker='.', linestyle=''
            # x_lim=lidar_max_range, y_lim=lidar_max_range,
        )

        self.distance_plot = RobotPlot(
            "distance",
            x_range=(0, 5), y_range=(0, 5), range_offset=1,
        )

        self.time_plot1 = RobotPlot("base time")
        self.prev_time1 = 0
        self.time_plot2 = RobotPlot("lidar time")
        self.prev_time2 = 0
        self.time_plots = RobotPlotCollection("time", self.time_plot1, self.time_plot2, plot_enabled=True)

        self.enable_slam = enable_slam
        self.cloud_updated = True

        self.sensor_id = "lidarsensor"
        self.base_id = "lidarbase"

        super(LidarTurret, self).__init__(self.sensor_id, self.base_id)

    def receive(self, timestamp, whoiam, packet):
        if whoiam == self.sensor_id:
            self.distance_parse(timestamp, packet)
        elif whoiam == self.base_id:
            self.tick_parse(timestamp, packet)

    def distance_parse(self, timestamp, packet):
        data = packet.split("\t")
        # self.dist_timestamp += int(data[1]) * 1E-6
        self.distances.append(int(data[0]))
        self.ticks.append(self.current_tick)

        self.time_plot2.append(timestamp, timestamp - self.prev_time2)
        self.prev_time2 = timestamp

    def tick_parse(self, timestamp, packet):
        data = packet.split("\t")
        self.time_plot1.append(timestamp, timestamp - self.prev_time1)

        if len(data) >= 2:
            self.current_tick += int(data[0])
            self.tick_timestamp += int(data[1]) * 1E-6

        if len(data) == 3:
            self.rotations = int(data[2])
            self.ticks_per_rotation = self.current_tick
            self.prev_time = timestamp

            self.update_rate_hz = 1 / (timestamp - self.prev_hz_time)
            self.prev_hz_time = timestamp
            print("Rotation #%4.0d @ %5.2fs (%4.2fHz), Ticks: %s, Points: %s" % (
                self.rotations, timestamp, self.update_rate_hz, self.current_tick, self.distances.index))
            self.current_tick = 0

            self.cloud_updated = True
            self.make_point_cloud()
            self.update_plots()

        self.prev_time1 = timestamp

    def make_point_cloud(self):
        if self.rotations > 2:
            self.distances.cap()
            self.ticks.cap()

            distances = self.distances.get()
            ticks = self.ticks.get()
            print(len(distances))
            for index in range(len(distances)):
                # while timestamp - ticks[tick_index][1] > 0:
                #     print(distance, timestamp, ticks[tick_index][1])

                angle = ticks[index] / self.ticks_per_rotation * 2 * math.pi
                x = distances[index] * math.cos(angle)
                y = distances[index] * math.sin(angle)

                self.point_cloud_xs.append(x)
                self.point_cloud_ys.append(y)

            self.point_cloud_xs.cap()
            self.point_cloud_ys.cap()

    def update_plots(self):
        self.point_cloud_plot.update(
            self.point_cloud_xs.get(),
            self.point_cloud_ys.get()
        )

    @property
    def point_cloud(self):
        return self.point_cloud_xs.get(), self.point_cloud_ys.get()

    def did_cloud_update(self):
        if self.cloud_updated:
            self.cloud_updated = False
            return True
        else:
            return False
