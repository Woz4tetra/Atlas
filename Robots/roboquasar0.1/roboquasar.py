import math

from algorithms.bozo_controller import BozoController
from algorithms.bozo_filter import BozoFilter
from algorithms.kalman.kalman_constants import constants
from algorithms.kalman.kalman_filter import GrovesKalmanFilter
from algorithms.pipeline import Pipeline, PipelinePID
from algorithms.pipeline2 import Pipeline2

from atlasbuggy.plotters.collection import RobotPlotCollection
from atlasbuggy.plotters.plot import RobotPlot

from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.plotters.staticplotter import StaticPlotter

from atlasbuggy.robot import Robot

from atlasbuggy.vision.camera import Camera

from sensors.gps import GPS
from sensors.imu import IMU
from sensors.lidarturret import LidarTurret

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow


class RoboQuasar(Robot):
    def __init__(self, enable_plotting, map_set_name,
                 initial_compass=None, animate=True, enable_cameras=True,
                 enable_kalman=False, use_log_file_maps=True, show_cameras=None,
                 pipeline=1, day_mode=False):

        # ----- initialize robot objects -----
        self.gps = GPS()
        self.imu = IMU()
        self.turret = LidarTurret(enabled=False)

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        super(RoboQuasar, self).__init__(
            self.gps, self.imu, self.turret, self.steering, self.brakes, self.underglow,
        )

        self.manual_mode = True

        # ----- init CV classes ------
        if show_cameras is None:
            show_cameras = enable_plotting

        self.left_camera = Camera("leftcam", enabled=enable_cameras, show=show_cameras)
        self.right_camera = Camera("rightcam", enabled=False, show=show_cameras)
        self.pipeline_pid = None

        self.day_mode = day_mode
        if pipeline == 1:
            self.left_pipeline = Pipeline(self.left_camera, self.day_mode, separate_read_thread=False)
            self.right_pipeline = Pipeline(self.right_camera, self.day_mode, separate_read_thread=False)
        elif pipeline == 2:
            self.left_pipeline = Pipeline2(self.left_camera, separate_read_thread=False)
            self.right_pipeline = Pipeline2(self.right_camera, separate_read_thread=False)
        self.pipeline_angle = 0.0

        # ----- init filters and controllers
        # position state message (in or out of map)
        self.position_state = ""
        self.prev_pos_state = self.position_state

        # init controller
        self.map_set_name = map_set_name
        print("Using map set:", map_set_name)
        checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets[map_set_name]

        self.controller = BozoController(checkpoint_map_name, map_dir, inner_map_name, outer_map_name, offset=5)
        self.bozo_filter = BozoFilter(initial_compass)
        self.use_log_file_maps = use_log_file_maps
        self.controller_angle = 0.0
        self.controller_weight_modifier = 1.0

        # who's controlling steering? (GPS and IMU or CV)
        self.gps_imu_control_enabled = True

        self.enable_kalman = enable_kalman
        self.kalman_filter = None
        self.imu_t0 = 0.0
        self.gps_t0 = 0.0

        # ----- link callbacks -----
        self.link_object(self.gps, self.receive_gps)
        self.link_object(self.imu, self.receive_imu)

        self.link_reoccuring(0.008, self.steering_event)
        self.link_reoccuring(0.05, self.update_auto_steering)
        self.link_reoccuring(0.05, self.brake_ping)

        # ----- init plots -----
        self.quasar_plotter = RoboQuasarPlotter(animate, enable_plotting, enable_kalman,
                                                self.controller.map, self.controller.inner_map,
                                                self.controller.outer_map, self.key_press, self.map_set_name)

    def init_kalman(self, timestamp, lat, long, altitude, initial_yaw, initial_pitch, initial_roll):
        if not self.enable_kalman:
            return
        self.kalman_filter = GrovesKalmanFilter(
            initial_roll=initial_roll,
            initial_pitch=initial_pitch,
            initial_yaw=initial_yaw,
            initial_lat=lat,
            initial_long=long,
            initial_alt=altitude,
            **constants
        )

        self.imu_t0 = timestamp
        self.gps_t0 = timestamp

    def start(self):
        # extract camera file name from current log file name
        file_name = self.get_path_info("file name no extension").replace(";", "_")
        directory = self.get_path_info("input dir")

        # start cameras and pipelines
        self.open_cameras(file_name, directory, "mp4")
        self.left_pipeline.start()
        self.right_pipeline.start()

        # record important data
        self.record("map set", self.map_set_name)
        self.record("initial compass", self.bozo_filter.compass_angle_packet)
        self.record("pipeline mode", str(int(self.day_mode)))

        self.pipeline_pid = PipelinePID(self.steering.left_limit_angle, self.steering.right_limit_angle, 0.0)

        self.debug_print("Using %s mode pipelines" % ("day" if self.day_mode else "night"), ignore_flag=True)
        if self.bozo_filter.initialized:
            self.debug_print("initial offset: %0.4f rad" % self.bozo_filter.compass_angle, ignore_flag=True)

    def open_cameras(self, log_name, directory, file_format):
        # create log name
        left_cam_name = "%s%s.%s" % (log_name, self.left_camera.name, file_format)
        right_cam_name = "%s%s.%s" % (log_name, self.right_camera.name, file_format)

        if self.logger is None:
            record = True
        else:
            record = self.logger.is_open()

        # launch a live camera if the robot is live, otherwise launch a video
        if self.is_live:
            status = self.left_camera.launch_camera(
                left_cam_name, directory, record,
                capture_number=1
            )
            if status is not None:
                return status

            status = self.right_camera.launch_camera(
                right_cam_name, directory, record,
                capture_number=2
            )
            if status is not None:
                return status
        else:
            self.left_camera.launch_video(left_cam_name, directory, start_frame=0)
            self.right_camera.launch_video(right_cam_name, directory)

    def receive_gps(self, timestamp, packet, packet_type):
        if self.gps.is_position_valid():
            if self.enable_kalman and self.bozo_filter.initialized:
                if self.kalman_filter is None:  # init kalman filter if initial heading is set and its enabled
                    self.init_kalman(timestamp,
                                     self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude,
                                     self.bozo_filter.compass_angle, 0.0, 0.0)
                else:  # otherwise update the filter
                    self.kalman_filter.gps_updated(
                        timestamp - self.gps_t0,
                        self.gps.latitude_deg, self.gps.longitude_deg, self.gps.altitude
                    )
                    self.gps_t0 = timestamp

                # display filter output
                self.quasar_plotter.kalman_recomputed_plot.append(*self.kalman_filter.get_position())
                self.record("kalman recorded", str(self.kalman_filter).replace("\n", "\t"))

            self.bozo_filter.update_bearing(self.gps.latitude_deg, self.gps.longitude_deg)
            if not self.controller.is_initialized():
                self.controller.initialize(self.gps.latitude_deg, self.gps.longitude_deg)

            status = self.quasar_plotter.update_gps_plot(self.dt(), self.gps.latitude_deg, self.gps.longitude_deg)
            if status is not None:
                return status

            outer_state = self.controller.point_inside_outer_map(self.gps.latitude_deg, self.gps.longitude_deg)
            inner_state = self.controller.point_outside_inner_map(self.gps.latitude_deg, self.gps.longitude_deg)

            bound_status = self.quasar_plotter.update_map_colors(self.dt(), outer_state, inner_state)
            if bound_status > 0:
                self.controller_weight_modifier = 0.5
            else:
                self.controller_weight_modifier = 1.0

    def receive_imu(self, timestamp, packet, packet_type):
        self.quasar_plotter.imu_plot.append(timestamp, self.imu.accel.x, self.imu.accel.y)

        if self.gps.is_position_valid() and self.bozo_filter.initialized:
            if self.enable_kalman:
                if self.kalman_filter is not None:
                    self.kalman_filter.imu_updated(
                        timestamp - self.imu_t0,
                        self.imu.accel.x, self.imu.accel.y, self.imu.accel.z,
                        self.imu.gyro.x, self.imu.gyro.y, self.imu.gyro.z,
                    )
                    self.imu_t0 = timestamp

                    lat, long = self.kalman_filter.get_position()[0:2]
                    filtered_angle = self.kalman_filter.get_orientation()[2]

                    imu_angle = self.bozo_filter.offset_angle(self.imu.euler.z)

                    if filtered_angle - imu_angle > math.pi:
                        imu_angle += 2 * math.pi
                    if imu_angle - filtered_angle > math.pi:
                        filtered_angle += 2 * math.pi

                    filtered_angle = 0.5 * filtered_angle + 0.5 * imu_angle
                else:
                    return
            else:
                filtered_angle = self.bozo_filter.offset_angle(self.imu.euler.z)
                lat, long = self.gps.latitude_deg, self.gps.longitude_deg

            self.controller_angle = self.controller.update(lat, long, filtered_angle)

            # if not self.manual_mode and self.gps_imu_control_enabled:
            #     self.steering.set_position(self.controller_angle)

            self.quasar_plotter.update_indicators(
                self.controller.current_pos, filtered_angle,
                self.bozo_filter.imu_angle,
                lat, long, self.controller_angle,
                self.controller.map[self.controller.current_index][0],
                self.controller.map[self.controller.current_index][1]
            )

    def received(self, timestamp, whoiam, packet, packet_type):
        self.left_pipeline.update_time(self.dt())

        if self.did_receive("initial compass"):
            self.bozo_filter.init_compass(packet)
            self.debug_print("initial offset: %0.4f rad" % self.bozo_filter.compass_angle, ignore_flag=True)

        elif self.did_receive("maps"):
            if self.use_log_file_maps:
                if "," in packet:
                    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = packet.split(",")
                else:
                    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets[packet]
                    self.quasar_plotter.plot_image(packet)
                self.controller.init_maps(checkpoint_map_name, map_dir, inner_map_name, outer_map_name)

                self.quasar_plotter.update_maps(self.controller.map, self.controller.inner_map,
                                                self.controller.outer_map)

        elif self.did_receive("checkpoint"):
            if self.gps.is_position_valid():
                self.quasar_plotter.checkpoint_plot.append(self.gps.latitude_deg, self.gps.longitude_deg)

        elif self.did_receive("steering angle"):
            if self.gps.is_position_valid():
                goal_index = int(packet.split("\t")[-1])
                self.quasar_plotter.plot_recorded_goal(self.gps.latitude_deg, self.gps.longitude_deg,
                                                       self.controller.map[goal_index][0],
                                                       self.controller.map[goal_index][1])

        # elif self.did_receive("kalman recorded"):
        #     position, orientation, velocity, attitude = packet.split("\t")
        #     position = [float(x[5:]) for x in position.split(", ")]
        #     self.quasar_plotter.kalman_recorded_plot.append(*position)

        elif self.did_receive("maps"):
            checkpoint_map_name, inner_map_name, outer_map_name, map_dir = packet.split(",")
            self.controller.init_maps(checkpoint_map_name, map_dir, inner_map_name, outer_map_name)

            self.quasar_plotter.update_maps(self.controller.map, self.controller.inner_map, self.controller.outer_map)
        elif self.did_receive("pipeline mode"):
            day_mode = bool(int(packet))
            self.left_pipeline.day_mode = day_mode
            self.right_pipeline.day_mode = day_mode
            self.debug_print("Switching to %s mode" % ("day" if day_mode else "night"), ignore_flag=True)

    def key_press(self, event):
        if event.key == " ":
            self.toggle_pause()
            if isinstance(self.quasar_plotter.plotter, LivePlotter):
                self.quasar_plotter.plotter.toggle_pause()

            self.left_pipeline.paused = not self.left_pipeline.paused
            self.right_pipeline.paused = not self.right_pipeline.paused
        elif event.key == "q":
            self.quasar_plotter.close("exit")
            self.close("exit")

    def loop(self):
        if self.is_paused:
            status = self.quasar_plotter.check_paused(self.dt())
            if status is not None:
                return status

        self.update_joystick()

    def update_auto_steering(self):
        left_percentage, status = self.get_safety_value(self.left_pipeline)
        if status == "error":
            return status
        right_percentage, status = self.get_safety_value(self.right_pipeline)
        if status == "error":
            return status

        # generate an angle from a PID
        self.pipeline_angle = self.pipeline_pid.update(self.dt(), left_percentage, right_percentage)

        # if not self.manual_mode and not self.gps_imu_control_enabled and self.pipeline_angle is not None:
        #     self.record("pipeline angle", self.pipeline_angle)
        #     # print(self.pipeline_angle)
        #     self.steering.set_position(self.pipeline_angle)
        if not self.manual_mode and self.pipeline_angle is not None and self.bozo_filter.initialized:
            if left_percentage != 0.0 and right_percentage != 0.0:
                # if trapped between two lines, average them to stay in the middle
                avg_percentage = (left_percentage + right_percentage) / 2
            elif left_percentage is not None:
                avg_percentage = left_percentage
            elif left_percentage is not None:
                avg_percentage = right_percentage
            else:
                avg_percentage = 0.0

            if left_percentage != 0.0 or right_percentage != 0.0:
                # weighted average. As the curb gets closer, the pipeline gains more control
                control_weight = avg_percentage * self.controller_weight_modifier
                pipeline_weight = 1 - avg_percentage * self.controller_weight_modifier

                steering_angle = control_weight * self.controller_angle + pipeline_weight * self.pipeline_angle
                # steering_angle = self.pipeline_angle
                self.steering.set_position(steering_angle)
            else:
                self.steering.set_position(self.controller_angle)

    def brake_ping(self):
        self.brakes.ping()

    def steering_event(self):
        if self.steering.calibrated and self.manual_mode:
            # if self.joystick.get_axis("ZR") >= 1.0:
            joy_val = self.joystick.get_axis("right x")
            if abs(joy_val) > 0.0:
                offset = math.copysign(0.3, joy_val)
                joy_val -= offset

            delta_step = self.my_round(16 * self.sigmoid(10.0 * joy_val))
            self.steering.change_step(delta_step)

    @staticmethod
    def my_round(x, d=0):
        p = 10 ** d
        return float(math.floor((x * p) + math.copysign(0.5, x))) / p

    @staticmethod
    def sigmoid(x):  # modified sigmoid. -1...1
        return (-1 / (1 + math.exp(-x)) + 0.5) * 2

    def update_joystick(self):
        if self.joystick is not None:
            if self.steering.calibrated and self.manual_mode:
                if self.joystick.axis_updated("left x") or self.joystick.axis_updated("left y"):
                    mag = math.sqrt(self.joystick.get_axis("left y") ** 2 + self.joystick.get_axis("left x") ** 2)
                    if mag > 0.5:
                        angle = math.atan2(self.joystick.get_axis("left y"), self.joystick.get_axis("left x"))
                        angle -= math.pi / 2
                        if angle < -math.pi:
                            angle += 2 * math.pi
                        self.steering.set_position(angle / 4)
                elif self.joystick.button_updated("A") and self.joystick.get_button("A"):
                    self.steering.calibrate()
                elif self.joystick.dpad_updated():
                    # if self.joystick.dpad[0] != 0:
                    #     self.steering.change_position(-self.joystick.dpad[0] * 10)
                    if self.joystick.dpad[1] != 0:
                        self.steering.set_position(0)

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")

            elif self.joystick.button_updated("L"):
                if self.joystick.get_button("L"):
                    self.brakes.release()
                    self.underglow.signal_release()
                else:
                    self.brakes.pull()
                    self.underglow.signal_brake()

            elif self.joystick.button_updated("R") and self.joystick.get_button("R"):
                self.brakes.toggle()
                if self.brakes.engaged:
                    self.underglow.signal_brake()
                else:
                    self.underglow.signal_release()

    def get_safety_value(self, pipeline):
        safety_percentage = 0.0
        if not self.manual_mode:
            if pipeline.did_update():
                safety_percentage = pipeline.safety_value

            if pipeline.status is not None:
                return safety_percentage, pipeline.status

            if pipeline.did_pause():
                self.toggle_pause()
                if isinstance(self.quasar_plotter.plotter, LivePlotter):
                    self.quasar_plotter.plotter.toggle_pause()

        return safety_percentage, None

    def close(self, reason):
        if reason != "done":
            self.brakes.pull()
            self.debug_print("!!EMERGENCY BRAKE!!", ignore_flag=True)

        self.quasar_plotter.close(reason)

        self.left_pipeline.close()
        self.right_pipeline.close()

        print("Ran for %0.4fs" % self.dt())


class RoboQuasarPlotter:
    def __init__(self, animate, enable_plotting, enable_kalman, course_map, inner_map, outer_map, key_press_fn,
                 map_set_name):
        # GPS map based plots
        self.gps_plot = RobotPlot("gps", color="red", enabled=True)
        self.map_plot = RobotPlot("map", color="purple")
        self.inner_map_plot = RobotPlot("inner map", enabled=True)
        self.outer_map_plot = RobotPlot("outer map", enabled=True)

        # angle based plots
        self.compass_plot = RobotPlot("quasar filtered heading", color="purple")
        self.compass_plot_imu_only = RobotPlot("imu filtered heading", color="magenta")
        self.sticky_compass_plot = RobotPlot("sticky compass", color="gray", enabled=False)
        self.steering_plot = RobotPlot("steering angle", color="gray", enabled=False)

        # indicator dots and lines
        self.checkpoint_plot = RobotPlot("checkpoint", color="green", marker='.', linestyle='', markersize=8)
        self.goal_plot = RobotPlot("checkpoint goal", color="cyan")
        self.recorded_goal_plot = RobotPlot("recorded checkpoint goal", "blue")
        self.current_pos_dot = RobotPlot("current pos dot", "blue", marker='.', markersize=8)

        # kalman plots
        self.enable_kalman = enable_kalman
        self.kalman_recomputed_plot = RobotPlot("kalman recomputed", color="blue", enabled=self.enable_kalman)
        self.kalman_recorded_plot = RobotPlot("kalman recorded", color="gray", enabled=self.enable_kalman)

        self.sticky_compass_counter = 0
        self.sticky_compass_skip = 100

        # plot collection
        self.accuracy_check_plot = RobotPlotCollection(
            "RoboQuasar plot", self.sticky_compass_plot, self.gps_plot,
            self.checkpoint_plot,
            self.map_plot, self.inner_map_plot, self.outer_map_plot,
            self.steering_plot, self.compass_plot_imu_only,
            self.compass_plot, self.goal_plot,
            self.current_pos_dot, self.kalman_recomputed_plot, self.kalman_recorded_plot,
        )

        # raw IMU data plot
        self.imu_plot = RobotPlot("imu data", flat=False, enabled=False)

        if animate:
            self.plotter = LivePlotter(
                2, self.accuracy_check_plot, self.imu_plot,
                matplotlib_events=dict(key_press_event=key_press_fn),
                enabled=enable_plotting, active_window_resizing=False
            )
        else:
            self.plotter = StaticPlotter(
                1, self.accuracy_check_plot,
                matplotlib_events=dict(key_press_event=key_press_fn),
                enabled=enable_plotting
            )

        # plot maps
        self.update_maps(course_map, inner_map, outer_map)
        self.plot_image(map_set_name)

        self.position_state = ""
        self.prev_pos_state = self.position_state

    def update_maps(self, course_map, inner_map, outer_map):
        self.map_plot.update(course_map.lats, course_map.longs)
        self.inner_map_plot.update(inner_map.lats, inner_map.longs)
        self.outer_map_plot.update(outer_map.lats, outer_map.longs)

    def plot_image(self, map_set_name):
        if map_set_name in image_sets:
            image_info = image_sets[map_set_name]
        else:
            return

        # image_path, img_point_1, img_point_2, gps_point_1, gps_point_2 = image_info
        self.plotter.draw_image(*image_info)

        # center = (gps_point_1[0] + gps_point_2[0]) / 2
        # width = abs(gps_point_1[1] - gps_point_2[1])

        # print([center - width, center + width], [gps_point_1[1], gps_point_2[1]])
        # self.image_box.update([center - width, center - width, center + width, center + width],
        #                       [gps_point_1[1], gps_point_2[1], gps_point_2[1], gps_point_1[1]])
        # self.plotter.get_axis(self.accuracy_check_plot).set_xlim((center - width, center + width))

    def update_gps_plot(self, dt, lat, long):
        if self.plotter.enabled:
            self.gps_plot.append(lat, long)

            if isinstance(self.plotter, LivePlotter):
                status = self.plotter.plot(dt)
                if status is not None:
                    return status

    def update_map_colors(self, dt, outer_state, inner_state):
        self.prev_pos_state = self.position_state
        status = -1
        if outer_state and inner_state:
            self.position_state = "in bounds"
            self.outer_map_plot.set_properties(color="blue")
            self.inner_map_plot.set_properties(color="blue")
            status = 0
        elif not outer_state:
            self.position_state = "out of bounds! (outer)"
            self.outer_map_plot.set_properties(color="red")
            status = 1
        elif not inner_state:
            self.position_state = "out of bounds! (inner)"
            self.inner_map_plot.set_properties(color="red")
            status = 2

        if self.position_state != self.prev_pos_state:
            print("%0.4f: %s" % (dt, self.position_state))

        return status

    def update_indicators(self, current_pos, filtered_angle, imu_angle, lat, long, steering_angle,
                          goal_lat, goal_long):
        if self.plotter.enabled:
            self.current_pos_dot.update([current_pos[0]], [current_pos[1]])
            if isinstance(self.plotter, LivePlotter):
                lat2, long2 = self.compass_coords(lat, long, filtered_angle)
                lat3, long3 = self.compass_coords(lat, long, imu_angle)

                self.compass_plot.update([lat, lat2],
                                         [long, long2])
                self.compass_plot_imu_only.update([lat, lat3],
                                                  [long, long3])
                self.plotter.draw_text(
                    self.accuracy_check_plot,
                    "%0.4f" % filtered_angle,
                    lat, long,
                    text_name="angle text"
                )

            if self.sticky_compass_skip > 0 and self.sticky_compass_counter % self.sticky_compass_skip == 0:
                lat2, long2 = self.compass_coords(lat, long, filtered_angle, length=0.0001)
                self.sticky_compass_plot.append(lat, long)
                self.sticky_compass_plot.append(lat2, long2)
                self.sticky_compass_plot.append(lat, long)
            self.sticky_compass_counter += 1

            if isinstance(self.plotter, LivePlotter):
                lat2, long2 = self.compass_coords(lat, long, steering_angle + filtered_angle)
                self.steering_plot.update([lat, lat2],
                                          [long, long2])

                self.goal_plot.update(
                    [lat, goal_lat],
                    [long, goal_long])

    @staticmethod
    def compass_coords(lat, long, angle, length=0.0003):
        lat2 = length * math.sin(-angle) + lat
        long2 = length * math.cos(-angle) + long
        return lat2, long2

    def plot_recorded_goal(self, lat, long, goal_lat, goal_long):
        if isinstance(self.plotter, LivePlotter):
            self.recorded_goal_plot.update(
                [lat, goal_lat],
                [long, goal_long]
            )

    def check_paused(self, dt):
        if isinstance(self.plotter, LivePlotter):
            status = self.plotter.plot(dt)
            if status is not None:
                return status

    def close(self, reason):
        if reason == "done":
            if isinstance(self.plotter, LivePlotter):
                print("done!")
                self.plotter.freeze_plot()

        self.plotter.close()


map_sets = {
    "single": (
        "Single Point",
        "Single Point Inside",
        "Single Point Outside",
        "single"
    ),
    "cut 3" : (
        "Autonomous Map 3",
        "Autonomous Map 3 Inner",
        "Autonomous Map 3 Outer",
        "cut"
    ),
    "cut 2" : (
        "Autonomous test map 2",
        "Autonomous test map 2 inside border",
        "Autonomous test map 2 outside border",
        "cut"
    ),
    "buggy" : (
        "buggy course map",
        "buggy course map inside border",
        "buggy course map outside border",
        "buggy",
    ),
    "short" : (
        "Short Course",
        "Short Course Inner",
        "Short Course Outer",
        "short"
    )
}

image_sets = {
    # "buggy": (
    #     "maps/buggy/Buggy Course Image 1.png", (40.440829, -79.948150),
    #     (40.441729, -79.941543), (136, 515), (2080, 161)
    # ),
    "buggy": (
        "maps/buggy/Buggy Course Image 2.png",
        (427, 2040), (2611, 836),
        (40.43910438921088, -79.94750440120698),
        (40.44156221153447, -79.94164109230043),
    ),
}

file_sets = {
    "rolls day 5"       : (
        ("07;46", "rolls/2017_Mar_26"),
        ("07;49", "rolls/2017_Mar_26"),
        ("07;54", "rolls/2017_Mar_26"),
        ("06;50", "rolls/2017_Mar_26"),  # first run
        ("06;55", "rolls/2017_Mar_26"),  # without camera
        ("06;56", "rolls/2017_Mar_26"),
        ("09;06", "rolls/2017_Mar_26"),
    ),
    "rolls day 4"       : (
        ("06;19", "rolls/2017_Mar_25"),  # 0, first semi-autonomous run part 1
        ("06;39", "rolls/2017_Mar_25"),  # 1, no initial compass
        ("06;48", "rolls/2017_Mar_25"),  # 2, first semi-autonomous run part 2
    ),
    "push practice 2"   : (
        ("23;55", "push_practice/2017_Mar_23"),  # 0, manual run
        ("23;57", "push_practice/2017_Mar_23"),  # 1, manual run
        ("00;10", "push_practice/2017_Mar_24"),  # 2, walking to position
        ("00;14", "push_practice/2017_Mar_24"),  # 3, manual run, heading is off
        ("00;17", "push_practice/2017_Mar_24"),  # 4, walking to position
        ("00;19", "push_practice/2017_Mar_24"),  # 5, FIRST AUTONOMOUS RUN!!!
        ("00;38", "push_practice/2017_Mar_24"),  # 6, AUTONOMOUS RUN!!! Hills 3, 4, & 5, ends in brake command failure
        ("00;45", "push_practice/2017_Mar_24"),  # 7, Finishing hill 5

        ("23;28", "push_practice/2017_Mar_23/error_logs"),  # 8, failed to send command brakes
        ("23;35", "push_practice/2017_Mar_23/error_logs"),  # 9, arduinos still running. Can't reconnect
        ("00;36", "push_practice/2017_Mar_24/error_logs"),  # 10, failed to send command steering
        ("00;44;47", "push_practice/2017_Mar_24/error_logs"),  # 11, arduinos still running. Can't reconnect
        ("00;44;58", "push_practice/2017_Mar_24/error_logs"),  # 12, failed to send command steering
        ("00;47", "push_practice/2017_Mar_24/error_logs"),  # 13, failed to send command brakes
        ("00;48;23", "push_practice/2017_Mar_24/error_logs"),  # 14, arduinos still running. Can't reconnect
        ("00;48;31", "push_practice/2017_Mar_24/error_logs"),  # 15, failed to send command steering
        ("00;50", "push_practice/2017_Mar_24/error_logs"),  # 16, failed to send command brakes
        ("00;51", "push_practice/2017_Mar_24/error_logs"),  # 17, failed to send command steering
        ("00;52;03", "push_practice/2017_Mar_24/error_logs"),  # 18, failed to send command brakes
        ("00;52;12", "push_practice/2017_Mar_24/error_logs"),  # 19, failed to send command brakes
    ),
    "push practice 1"   : (
        ("23;25", "push_practice/2017_Mar_21"),
        ("23;30", "push_practice/2017_Mar_21"),
        ("23;49", "push_practice/2017_Mar_21"),
        ("23;55", "push_practice/2017_Mar_21"),
        ("00;06", "push_practice/2017_Mar_22"),
        ("00;15", "push_practice/2017_Mar_22"),
    ),
    "data day 12"       : (
        ("16;36", "data_days/2017_Mar_18"),
    ),
    "data day 11"       : (
        ("16;04", "data_days/2017_Mar_13"),
        ("16;31", "data_days/2017_Mar_13"),
        ("16;19", "data_days/2017_Mar_13"),
    ),
    "data day 10"       : (
        # straight line GPS tests
        ("17;07", "data_days/2017_Mar_09"),
        ("17;09", "data_days/2017_Mar_09"),
        ("17;11", "data_days/2017_Mar_09"),

        # going to single point (initial angle: 235)
        ("17;29", "data_days/2017_Mar_09"),
        ("17;33", "data_days/2017_Mar_09"),
        ("17;36", "data_days/2017_Mar_09"),
        ("17;38", "data_days/2017_Mar_09"),
        ("17;41", "data_days/2017_Mar_09"),
        ("17;44", "data_days/2017_Mar_09"),
        ("17;46", "data_days/2017_Mar_09"),
        ("17;49", "data_days/2017_Mar_09"),

        ("17;55", "data_days/2017_Mar_09"),  # autonomous run

        # gently crashing on the buggy course
        ("18;07", "data_days/2017_Mar_09"),
        ("18;09", "data_days/2017_Mar_09"),

        # cables disconnecting walking home
        ("18;30;51", "data_days/2017_Mar_09"),
        ("18;36", "data_days/2017_Mar_09"),
    ),
    "data day 9"        : (
        ("15;13", "data_days/2017_Mar_05"),  # 0
        ("15;19", "data_days/2017_Mar_05"),  # 1
        ("15;25", "data_days/2017_Mar_05"),  # 2
        ("15;27", "data_days/2017_Mar_05"),  # 3, running into the lamp post
        ("15;37", "data_days/2017_Mar_05"),  # 4, !! Weird errors !! not sure what's going on
        ("15;46", "data_days/2017_Mar_05"),  # 5
        ("16;00", "data_days/2017_Mar_05"),  # 6, Sketchy run
        ("16;05", "data_days/2017_Mar_05"),  # 7
        ("16;06", "data_days/2017_Mar_05"),  # 8, Weird error again, pointing 180 degrees the wrong way
        ("16;11", "data_days/2017_Mar_05"),  # 9, Steering severely limited
        ("16;29", "data_days/2017_Mar_05"),  # 10, Walking home. UC offered interesting interference
    ),
    "moving to high bay": (
        ("14;08;26", "data_days/2017_Mar_02"),
    ),
    "data day 8"        : (
        ("15;05", "data_days/2017_Feb_26"),  # 0, 1st autonomous run
        ("15;11", "data_days/2017_Feb_26"),  # 1, 2nd autonomous run, IMU stopped working
        ("15;19", "data_days/2017_Feb_26"),  # 2, finished 2nd run
        ("15;23", "data_days/2017_Feb_26"),  # 3, 3rd run, very good run. Did multiple laps. Joystick out of range test
        ("15;33", "data_days/2017_Feb_26"),  # 4, 4th run, multiple laps
        ("15;43", "data_days/2017_Feb_26"),  # 5, walking home
    ),
    "data day 7"        : (
        ("14;19", "data_days/2017_Feb_24"),  # 0, rolling on schlenley 1
        ("14;26", "data_days/2017_Feb_24"),  # 1, rolling on schlenley 2
        ("16;13", "data_days/2017_Feb_24"),  # 2, GPS not found error 1
        ("16;14", "data_days/2017_Feb_24"),  # 3, Moving to the cut part 1
        ("16;21", "data_days/2017_Feb_24"),  # 4, Moving to the cut part 2
        ("16;49", "data_days/2017_Feb_24"),  # 5, GPS not found error 2
        ("17;09", "data_days/2017_Feb_24"),  # 6, Faulty checkpoint lock ons
        ("17;33", "data_days/2017_Feb_24"),  # 7, Manual roll around course
        ("20;06", "data_days/2017_Feb_24"),  # 8, Walking home
        ("FIRST SUCCESSFUL RUN", "data_days/2017_Feb_24"),
    ),

    # started using checkpoints
    "data day 6"        : (
        ("16;47", "data_days/2017_Feb_18"),
        ("16;58", "data_days/2017_Feb_18"),
        ("18;15", "data_days/2017_Feb_18"),
    ),
    "data day 5"        : (
        ("16;49", "data_days/2017_Feb_17"),
        ("17;37", "data_days/2017_Feb_17"),
        ("18;32", "data_days/2017_Feb_17"),
    ),
    # "rolls day 4": (
    #
    # ),
    "data day 4"        : (
        ("15", "data_days/2017_Feb_14"),  # filter explodes, LIDAR interfered by the sun
        ("16;20", "data_days/2017_Feb_14"),  # shorten run, LIDAR collapsed
        ("16;57", "data_days/2017_Feb_14"),  # interfered LIDAR
        ("17;10", "data_days/2017_Feb_14"),  # all data is fine, interfered LIDAR
        ("17;33", "data_days/2017_Feb_14")),  # data is fine, normal run

    "data day 3"        : (
        ("16;38", "data_days/2017_Feb_08"),
        ("17", "data_days/2017_Feb_08"),
        ("18", "data_days/2017_Feb_08")),

    # no gyro values
    "trackfield"        : (
        ("15;46", "old_data/2016_Dec_02"),
        ("15;54", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02"),
        ("16;10", "old_data/2016_Dec_02")),

    "rolls day 1"       : (
        ("07;22", "old_data/2016_Nov_06"),),  # bad gyro values

    "rolls day 2"       : (
        ("07;36;03 m", "old_data/2016_Nov_12"),
        ("09;12", "old_data/2016_Nov_12"),  # invalid values
        ("07;04;57", "old_data/2016_Nov_13")),  # bad gyro values

    "rolls day 3"       : (
        ("modified 07;04", "old_data/2016_Nov_13"),
        ("modified 07;23", "old_data/2016_Nov_13")),  # wonky value for mag.

    # rolling on the cut
    "first cut test"    : (
        ("16;29", "old_data/2016_Dec_09"),
        ("16;49", "old_data/2016_Dec_09"),
        ("16;5", "old_data/2016_Dec_09"),
        ("17;", "old_data/2016_Dec_09")),  # nothing wrong, really short

    "bad data"          : (
        ("16;07", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("16;09", "old_data/2016_Dec_09/bad_data"),  # nothing wrong, really short
        ("18;00", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;02", "old_data/2016_Dec_09/bad_data"),  # gps spazzed out
        ("18;09", "old_data/2016_Dec_09/bad_data")),  # gps spazzed out
}

video_sets = {
    "data day 12": (
        ("16_36_17", "data_days/2017_Mar_18", "mp4"),
    ),
    "data day 11": (
        ("17_00_00", "data_days/2017_Mar_16", "mp4"),  # 0
        ("17_01_40", "data_days/2017_Mar_16", "mp4"),  # 1
        ("17_05_21", "data_days/2017_Mar_16", "mp4"),  # 2
        ("17_06_50", "data_days/2017_Mar_16", "mp4"),  # 3
        ("17_10_16", "data_days/2017_Mar_16", "mp4"),  # 4
        ("17_12_45", "data_days/2017_Mar_16", "mp4"),  # 5
        ("17_14_46", "data_days/2017_Mar_16", "mp4"),  # 6
        ("17_20_03", "data_days/2017_Mar_16", "mp4"),  # 7
        ("17_30_23", "data_days/2017_Mar_16", "mp4"),  # 8
        ("17_32_19", "data_days/2017_Mar_16", "mp4"),  # 9
        ("18_33_06", "data_days/2017_Mar_14", "avi"),  # 10
        ("18_34_26", "data_days/2017_Mar_14", "avi"),  # 11
        ("18_36_30", "data_days/2017_Mar_14", "avi"),  # 12
    )
}
