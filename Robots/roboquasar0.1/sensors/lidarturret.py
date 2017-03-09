import math

from atlasbuggy.robot.object import RobotObject
from atlasbuggy.plotters.plot import RobotPlot
from atlasbuggy.plotters.collection import RobotPlotCollection


class LidarTurret(RobotObject):
    def __init__(self, enabled=True, point_cloud_size=100,
                 angle_range=(210, 330), reverse_range=True,
                 # angle_range=(0, 360), reverse_range=False
                 ):
        # ----- point creation -----
        self._current_tick = 0
        self._ticks_per_rotation = 0
        self._prev_hz_time = 0
        self._update_rate_hz = 0.0

        self._raw_distances = DynamicList()
        self._distances = DynamicList()
        self._angles = DynamicList()
        self._ticks = DynamicList()

        self._rotations = 0

        self.raw_point_cloud_size = point_cloud_size

        assert angle_range[0] >= 0 and angle_range[1] >= 0
        assert angle_range[1] > angle_range[0]

        self.angle_range = angle_range
        min_index = int(self.angle_range[0] / 360 * point_cloud_size)
        max_index = int(self.angle_range[1] / 360 * point_cloud_size)

        self.index_range = min_index, max_index
        self.reverse_range = reverse_range
        if self.reverse_range:
            self.point_cloud_size = self.raw_point_cloud_size - int(max_index - min_index)
            self.detection_angle_degrees = 360 - int(self.angle_range[1] - self.angle_range[0])
        else:
            self.point_cloud_size = int(max_index - min_index)
            self.detection_angle_degrees = int(self.angle_range[1] - self.angle_range[0])

        # ----- point cloud formats -----

        self._raw_point_cloud_xs = DynamicList()
        self._raw_point_cloud_ys = DynamicList()

        self._point_cloud_xs = DynamicList()
        self._point_cloud_ys = DynamicList()

        self._point_cloud_x_lines = DynamicList()
        self._point_cloud_y_lines = DynamicList()

        # ----- plots -----

        lidar_range = (-100, 100)
        self.raw_point_cloud_plot = RobotPlot(
            "raw point cloud",
            # plot_enabled=False,
            x_range=lidar_range, y_range=lidar_range, range_offset=1,
            marker='.', linestyle=''
        )

        self.point_cloud_plot = RobotPlot(
            "point cloud",
            x_range=lidar_range, y_range=lidar_range, range_offset=1,
            marker='.', linestyle=''
        )

        self.line_plot = RobotPlot(
            "lines",
            enabled=False,
            x_range=lidar_range, y_range=lidar_range, range_offset=1,
            marker='', linestyle='-', color='lightblue',
            # skip_count=10
        )

        self.point_cloud_plot_collection = RobotPlotCollection("point cloud", self.raw_point_cloud_plot,
                                                               self.point_cloud_plot, self.line_plot)
        self.cloud_updated = False
        self.cloud_is_ready = False

        self.print_str = ""

        # ----- SLAM -----

        self.paused = False
        self.error_header = "> "

        super(LidarTurret, self).__init__("lidar", enabled)

    def receive(self, timestamp, packet):
        if packet[0] == 'l':
            data = packet[1:].split("\t")

            delta_tick = int(data[0])
            distance = int(data[1])  # millimeters

            self._current_tick += delta_tick

            self._raw_distances.append(distance)
            self._ticks.append(self._current_tick)
        elif packet[0] == 'r':
            self.update_properties(timestamp, packet)
            self.make_point_cloud()

            self.print_str = "Rotation #%4.0d @ %5.2fs (%4.2fHz), Ticks: %s, Points: %s, Interpolated: %s" % (
                self._rotations, timestamp, self._update_rate_hz, self._current_tick, len(self._raw_distances),
                len(self._point_cloud_xs))

            self._current_tick = 0
            self.cloud_updated = True

            if self._rotations > 5:
                self.cloud_is_ready = True

    def did_cloud_update(self):
        if self.cloud_updated:
            self.cloud_updated = False
            return True
        else:
            return False

    def is_cloud_ready(self):
        if self.cloud_is_ready:
            self.cloud_is_ready = False
            return True
        else:
            return False

    def update_properties(self, timestamp, packet):
        self._raw_distances.cap()
        self._ticks.cap()
        self._ticks_per_rotation = self._current_tick

        self._rotations = int(packet[1:])
        self._update_rate_hz = 1 / (timestamp - self._prev_hz_time)
        self._prev_hz_time = timestamp

    def make_point_cloud(self):
        if self._rotations > 2:

            # make angle array
            raw_increment = 2 * math.pi / self._ticks_per_rotation
            for raw_index in range(len(self._raw_distances)):
                angle = self._ticks[raw_index] * raw_increment
                self._angles.append(angle)

                if self.raw_point_cloud_plot.enabled:
                    x = self._raw_distances[raw_index] * math.cos(angle)
                    y = self._raw_distances[raw_index] * math.sin(angle)

                    self._raw_point_cloud_xs.append(x)
                    self._raw_point_cloud_ys.append(y)

            self._angles.cap()
            if self.raw_point_cloud_plot.enabled:
                self._raw_point_cloud_xs.cap()
                self._raw_point_cloud_ys.cap()

            # create point cloud. Interpolate missing points
            raw_index = 1
            cloud_increment = 2 * math.pi / self.raw_point_cloud_size
            for cloud_index in range(1, self.raw_point_cloud_size):
                desired_angle = cloud_index * cloud_increment

                # If there are more recorded points than desired, skip some
                # (more points than desired case)
                while self._angles[raw_index] < desired_angle:
                    raw_index += 1

                current_angle = self._angles[raw_index]
                prev_angle = self._angles[raw_index - 1]
                if prev_angle > current_angle:  # adjust bounds for interpolation
                    prev_angle -= 2 * math.pi

                if self.reverse_range:
                    should_interpolate = cloud_index <= self.index_range[0] or self.index_range[1] < cloud_index
                else:
                    should_interpolate = self.index_range[0] <= cloud_index < self.index_range[1]
                if should_interpolate:
                    self._interpolate_distance(
                        self._raw_distances[raw_index], self._raw_distances[raw_index - 1],
                        current_angle, prev_angle, desired_angle
                    )

                # if the next angle is still within the raw data, interpolate with the same range
                # (less points than desired case)
                if (cloud_index + 1) * cloud_increment > self._raw_distances[raw_index]:
                    raw_index += 1

            # interpolate last and first point (making up for skipped first iteration)
            self._interpolate_distance(self._raw_distances[-1], self._raw_distances[0], 2 * math.pi,
                                       self._ticks[-2] / self._ticks_per_rotation * 2 * math.pi, 2 * math.pi)
            # ----- update variables and plots -----
            if self.line_plot.enabled:
                self._point_cloud_x_lines.cap()
                self._point_cloud_y_lines.cap()

            self._point_cloud_xs.cap()
            self._point_cloud_ys.cap()

            self._distances.cap()

            self.raw_point_cloud_plot.update(self._raw_point_cloud_xs.get(), self._raw_point_cloud_ys.get())
            self.point_cloud_plot.update(self._point_cloud_xs.get(), self._point_cloud_ys.get())
            self.line_plot.update(self._point_cloud_x_lines.get(), self._point_cloud_y_lines.get())

    def _interpolate_distance(self, distance, prev_distance, angle, prev_angle, desired_angle):
        slope = (distance - prev_distance) / (angle - prev_angle)
        # if slope <= 1.0:
        interpolation = int(slope * (desired_angle - prev_angle) + prev_distance)
        # else:
        #     interpolation = distance

        interp_x = interpolation * math.cos(desired_angle)
        interp_y = interpolation * math.sin(desired_angle)

        self._point_cloud_xs.append(interp_x)
        self._point_cloud_ys.append(interp_y)

        self._distances.append(interpolation)

        if self.line_plot.enabled:
            self._point_cloud_x_lines.append(interp_x)
            self._point_cloud_y_lines.append(interp_y)
            self._point_cloud_x_lines.append(0)
            self._point_cloud_y_lines.append(0)

    @property
    def point_cloud(self):
        # (puts in (x0, y0), (x1, y1)... format)
        return zip(self._point_cloud_xs.get(), self._point_cloud_ys.get())

    @property
    def distances(self):
        return self._distances.get()

    def set_paused(self, paused=None):
        if paused is None:
            self.paused = not self.paused

        self.send("p%i" % int(self.paused))

    def __str__(self):
        return self.print_str


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

    def __setitem__(self, key, value):
        self.l[key % self.end_index] = value

    def __len__(self):
        return self.end_index
