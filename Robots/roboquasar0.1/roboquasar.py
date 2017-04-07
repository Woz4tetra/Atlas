import math
import time

from algorithms.bozo_controller import MapManipulator
from algorithms.bozo_filter import AngleManipulator
from algorithms.pipeline import Pipeline, PID

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
                 use_log_file_maps=True, show_cameras=None, day_mode=False):

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

        self.right_camera = Camera("rightcam", enabled=enable_cameras, show=show_cameras, )

        self.day_mode = day_mode
        self.right_pipeline = Pipeline(self.right_camera, self.day_mode, separate_read_thread=False)

        # ----- init filters and controllers
        # position state message (in or out of map)
        self.position_state = ""
        self.prev_pos_state = self.position_state

        # init controller
        self.map_set_name = map_set_name
        print("Using map set:", map_set_name)
        checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets[map_set_name]

        self.init_checkpoints = [
            0,  # hill 1
            117,  # hill 3
            123,  # hill 4
            139,  # hill 5
        ]
        self.initial_index = 0
        self.init_offset = 1
        self.map_manipulator = MapManipulator(checkpoint_map_name, map_dir, inner_map_name, outer_map_name, offset=3,
                                              init_indices=self.init_checkpoints)
        self.angle_filter = AngleManipulator(initial_compass)

        self.use_log_file_maps = use_log_file_maps

        # ----- filtered outputs -----
        self.imu_heading_guess = 0.0
        self.steering_angle = 0.0
        self.map_heading = 0.0

        self.angle_threshold = math.pi / 6
        self.percent_threshold = 0.4

        self.gps_within_map = True
        self.prev_gps_state = False
        self.uncertainty_condition = False
        self.prev_pipeline_value_state = False

        # ----- link callbacks -----
        self.link_object(self.gps, self.receive_gps)
        self.link_object(self.imu, self.receive_imu)

        self.link_reoccuring(0.008, self.steering_event)
        self.link_reoccuring(0.05, self.brake_ping)

        # ----- init plots -----
        self.quasar_plotter = RoboQuasarPlotter(animate, enable_plotting, False,
                                                self.map_manipulator.map, self.map_manipulator.inner_map,
                                                self.map_manipulator.outer_map, self.key_press, self.map_set_name)

        # ----- initializing heading ------
        self.heading_setup = 0
        self.initial_drift = (0, 0)
        self.initial_pos = (0, 0)
        self.abs_drift = (0, 0)
        self.calibration_positions = []

    def start(self):
        # extract camera file name from current log file name
        file_name = self.get_path_info("file name no extension").replace(";", "_")
        directory = self.get_path_info("input dir")

        # start cameras and pipelines
        self.open_cameras(file_name, directory, "avi")
        self.right_pipeline.start()

        # record important data
        self.record("map set", self.map_set_name)
        self.record_compass()
        if self.day_mode is not None:
            self.record("pipeline mode", str(int(self.day_mode)))

        self.debug_print("Using %s mode pipelines" % ("day" if self.day_mode else "night"), ignore_flag=True)

        self.brakes.unpause_pings()

    def open_cameras(self, log_name, directory, file_format):
        # create log name
        right_cam_name = "%s%s.%s" % (log_name, self.right_camera.name, file_format)

        if self.logger is None:
            record = True
        else:
            record = self.logger.is_open()

        # launch a live camera if the robot is live, otherwise launch a video
        if self.is_live:
            status = self.right_camera.launch_camera(
                right_cam_name, directory, record,
                capture_number=1
            )
            if status is not None:
                return status
        else:
            self.right_camera.launch_video(right_cam_name, directory)

    def receive_gps(self, timestamp, packet, packet_type):
        is_valid = self.gps.is_position_valid()
        if is_valid != self.prev_gps_state:
            self.prev_gps_state = is_valid
            if is_valid:
                print("GPS has locked on:")
                print(self.gps)
            else:
                print("WARNING. GPS has lost connection!!!")

        if is_valid:
            if self.heading_setup == 2:
                self.gps.latitude_deg += self.abs_drift[0]
                self.gps.longitude_deg += self.abs_drift[1]
            if not self.map_manipulator.is_initialized():
                self.map_manipulator.initialize(self.gps.latitude_deg, self.gps.longitude_deg)

            outer_state = self.map_manipulator.point_inside_outer_map(self.gps.latitude_deg, self.gps.longitude_deg)
            inner_state = self.map_manipulator.point_outside_inner_map(self.gps.latitude_deg, self.gps.longitude_deg)
            self.gps_within_map = outer_state and inner_state

            if self.heading_setup == 1 and not self.gps_within_map:
                print("WARNING. Drift correction has been applied but GPS is still out of the map.")

            self.quasar_plotter.update_map_colors(timestamp, outer_state, inner_state)

            self.map_heading = self.map_manipulator.get_goal_angle(
                self.gps.latitude_deg, self.gps.longitude_deg
            )

            self.quasar_plotter.goal_plot.update([self.gps.latitude_deg, self.map_manipulator.goal_lat],
                                                 [self.gps.longitude_deg, self.map_manipulator.goal_long])

            status = self.quasar_plotter.update_gps_plot(timestamp, self.gps.latitude_deg, self.gps.longitude_deg)
            if status is not None:
                return status

    def receive_imu(self, timestamp, packet, packet_type):
        if self.angle_filter.initialized and self.heading_setup == 2:
            self.imu_heading_guess = self.angle_filter.offset_angle(self.imu.euler.z)
            if self.map_manipulator.is_initialized():
                # assign steering angle from IMU regardless of the pipeline,
                # in case the pipeline finds it's too close but sees the buggy is straight. In this case, the pipeline
                # would not assign a new angle
                self.steering_angle = self.map_manipulator.update(
                    self.gps.latitude_deg, self.gps.longitude_deg, self.imu_heading_guess
                )

                if self.right_pipeline.safety_value > self.percent_threshold:
                    if not self.prev_pipeline_value_state:
                        print("Pipeline is taking over steering: %s" % self.right_pipeline.safety_value)
                        self.prev_pipeline_value_state = True

                    # if the pipelines say there is significant angle deviation but the IMU says we're going straight,
                    # adjust IMU offset using the pipeline and knowledge of current map position

                    # TODO: double check this range is correct Make sure it's not less than the hough detector limits
                    pipeline_condition = not (
                        -self.angle_threshold < self.right_pipeline.line_angle < self.angle_threshold)
                    imu_condition = -self.angle_threshold < self.steering_angle < self.angle_threshold

                    pipeline_angle = self.map_heading + self.right_pipeline.line_angle

                    if (pipeline_condition and imu_condition) or self.uncertainty_condition:
                        if not self.uncertainty_condition:
                            print("Uncertainty condition applied. Guiding with CV until straight")
                            print(
                                "Angles; CV: %s -> %s, IMU: %s, threshold: %s" % (
                                    self.right_pipeline.line_angle, pipeline_angle, self.steering_angle,
                                    self.angle_threshold)
                            )

                        self.uncertainty_condition = True

                        # use the pipeline and map offset for angle instead
                        self.steering_angle = self.map_manipulator.update(
                            self.gps.latitude_deg, self.gps.longitude_deg, pipeline_angle
                        )

                    # Keep adjusting the angle until the pipeline thinks the buggy is straight,
                    # then recalibrate IMU offset
                    if -self.angle_threshold / 2 < self.right_pipeline.line_angle < self.angle_threshold / 2:
                        self.uncertainty_condition = False
                        self.angle_filter.init_compass(pipeline_angle)
                        print("Buggy is straight. Recalibrating steering:", pipeline_angle)
                elif self.prev_pipeline_value_state:
                    print(
                        "IMU is taking over steering. Steering: %s, IMU: %s, Map: %s" % (
                            self.steering_angle, self.imu_heading_guess, self.map_heading)
                    )
                    self.prev_pipeline_value_state = False

            self.update_steering()

            if self.gps.is_position_valid():
                self.quasar_plotter.update_indicators((self.gps.latitude_deg, self.gps.longitude_deg),
                                                      self.imu_heading_guess)

    def received(self, timestamp, whoiam, packet, packet_type):
        # self.left_pipeline.update_time(self.dt())

        if self.did_receive("initial compass"):
            self.angle_filter.init_compass(packet)
            self.debug_print("initial offset: %0.4f rad" % self.angle_filter.compass_angle, ignore_flag=True)

        elif self.did_receive("maps"):
            if self.use_log_file_maps:
                if "," in packet:
                    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = packet.split(",")
                else:
                    checkpoint_map_name, inner_map_name, outer_map_name, map_dir = map_sets[packet]
                    self.quasar_plotter.plot_image(packet)
                self.map_manipulator.init_maps(checkpoint_map_name, map_dir, inner_map_name, outer_map_name)

                self.quasar_plotter.update_maps(self.map_manipulator.map, self.map_manipulator.inner_map,
                                                self.map_manipulator.outer_map)

        elif self.did_receive("pipeline mode"):
            day_mode = bool(int(packet))
            self.right_pipeline.day_mode = day_mode
            self.debug_print("Switching to %s mode" % ("day" if day_mode else "night"), ignore_flag=True)

    def update_steering(self):
        if self.angle_filter.initialized:
            left_limit = abs(self.steering.left_limit_angle / 3)
            if self.steering_angle > left_limit:  # never turn left too much
                self.steering_angle = left_limit

            self.quasar_plotter.update_goal_angle(
                self.gps.latitude_deg, self.gps.longitude_deg,
                self.imu_heading_guess - self.steering_angle
            )

            if not self.manual_mode:
                self.steering.set_position(self.steering_angle)
                # print("%0.4f, %0.4f -> %0.4f" % (self.pipeline_angle, self.imu_steering_angle, steering_angle))

    def key_press(self, event):
        if event.key == " ":
            self.toggle_pause()
            if isinstance(self.quasar_plotter.plotter, LivePlotter):
                self.quasar_plotter.plotter.toggle_pause()

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

            elif self.joystick.button_updated("Y") and self.joystick.get_button("Y"):
                #self.calibrate_with_checkpoints()
                self.calibrate_with_checkpoint()

    def calibrate_with_checkpoints(self):
        if self.gps.is_position_valid():
            if self.heading_setup == 0:
                self.heading_setup = 1
                self.initial_pos, self.initial_index = self.map_manipulator.lock_onto_map(
                    self.gps.latitude_deg, self.gps.longitude_deg, True
                )
                self.initial_drift = (self.initial_pos[0] - self.gps.latitude_deg,
                                      self.initial_pos[1] - self.gps.longitude_deg)
                self.num_points += 1

                print("----------")
                print("\tInitial GPS: (%0.6f, %0.6f)" % (self.gps.latitude_deg, self.gps.longitude_deg))
                print("\tLocked index: %s" % self.initial_index)
                print("\tDrift 1: (%0.6f, %0.6f)" % self.initial_drift)
            elif self.heading_setup == 1:
                # actual_pos = self.map_manipulator.lock_onto_map(gps_reading[0], gps_reading[1], True)
                actual_lat, actual_long = self.map_manipulator.map[self.initial_index + self.init_offset]
                second_drift = actual_lat - self.gps.latitude_deg, actual_long - self.gps.longitude_deg

                self.abs_drift = (self.initial_drift[0] + second_drift[0]) / 2, \
                                 (self.initial_drift[1] + second_drift[1]) / 2
                bearing = AngleManipulator.bearing_to(
                    self.initial_pos[0], self.initial_pos[1], actual_lat, actual_long
                )

                self.angle_filter.init_compass(bearing)
                self.record_compass()

                self.heading_setup = 2

                print("\n\tGPS: (%0.6f, %0.6f)" % (self.gps.latitude_deg, self.gps.longitude_deg))
                print("\tLocked GPS: (%0.6f, %0.6f)" % (actual_lat, actual_long))
                print("\tLocked index: %s" % (self.initial_index + self.init_offset))
                print("\tDrift 2: (%0.6f, %0.6f)" % second_drift)
                print("\tAvg drift: (%0.6f, %0.6f)" % self.abs_drift)
                print("\tBearing: %0.6f" % bearing)
                print("----------")

            elif self.heading_setup == 2:
            	prev_lat, prev_long = self.map_manipulator.lock_onto_map(
                    self.gps.latitude_deg, self.gps.longitude_deg, True
                )
            	self.initial_pos = prev_lat, prev_long
            	actual_lat, actual_long = self.map_manipulator.map[self.initial_index+self.init_offset]
            	self.num_points+= 1
            	nth_drift = actual_lat - self.gps.latitude_deg, actual_long - self.gps.longitude_deg
            	#new avg
            	self.abs_drift = self.abs_drift[0] + (nth_drift[0] - self.abs_drift[0])/self.num_points,
            					 self.abs_drift[1] + (nth_drift[1] - self.abs_drift[1])/self.num_points
            	bearing = AngleManipulator.bearing_to(
            		prev_lat, prev_lon, actual_lat, actual_lon
            	)
            	self.angle_filter.init_compass(bearing)
            	self.record_compass()

                print("\n\tGPS: (%0.6f, %0.6f)" % (self.gps.latitude_deg, self.gps.longitude_deg))
                print("\tLocked GPS: (%0.6f, %0.6f)" % (actual_lat, actual_long))
                print("\tLocked index: %s" % (self.initial_index + self.init_offset))
                print("\tDrift %d: (%0.6f, %0.6f)" % (self.num_points, nth_drift))
                print("\tNew avg drift: (%0.6f, %0.6f)" % self.abs_drift)
                print("\tBearing: %0.6f" % bearing)
                print("----------")
        else:
            print("Invalid GPS. Ignoring")

    def calibrate_with_checkpoint(self):
        if self.gps.is_position_valid():
            if self.heading_setup == 0:
                self.heading_setup = 1
                self.initial_pos, self.initial_index = self.map_manipulator.lock_onto_map(
                    self.gps.latitude_deg, self.gps.longitude_deg, True
                )
                self.calibration_positions.append(self.initial_pos)
                self.initial_drift = (self.initial_pos[0] - self.gps.latitude_deg,
                                      self.initial_pos[1] - self.gps.longitude_deg)
                next_pos = self.map_manipulator[self.initial_index + self.init_offset]
                bearing = AngleManipulator.bearing_to(
                	self.initial_pos[0], self.initial_pos[1], next_pos[0], next_pos[1])
                self.angle_filter.init_compass(bearing)
                self.record_compass()
                print("----------")
                print("\tInitial GPS: (%0.6f, %0.6f)" % (self.gps.latitude_deg, self.gps.longitude_deg))
                print("\tLocked index: %s" % self.initial_index)
                print("\tDrift: (%0.6f, %0.6f)" % self.initial_drift)
                print("----------")

            elif self.heading_setup == 1:
                # actual_pos = self.map_manipulator.lock_onto_map(gps_reading[0], gps_reading[1], True)
            	new_pos, self.initial_index = self.map_manipulator.lock_onto_map(
                    self.gps.latitude_deg, self.gps.longitude_deg, False
                )
            	self.calibration_positions.append(new_pos)
            	next_pos = self.map_manipulator.map[self.initial_index+self.init_offset]
            	nth_drift = new_pos[0] - self.gps.latitude_deg, new_pos[1] - self.gps.longitude_deg
            	#new avg
            	self.abs_drift = self.abs_drift[0] + (nth_drift[0] - self.abs_drift[0])/len(self.calibration_positions),\
            					 self.abs_drift[1] + (nth_drift[1] - self.abs_drift[1])/len(self.calibration_positions)
            	bearing = AngleManipulator.bearing_to(
            		new_pos[0], new_pos[1], next_pos[0], next_pos[1]
            	)
            	self.angle_filter.init_compass(bearing)
            	self.record_compass()
                print("----------")
                print("\n\tGPS: (%0.6f, %0.6f)" % (self.gps.latitude_deg, self.gps.longitude_deg))
                print("\tLocked GPS: (%0.6f, %0.6f)" % (actual_lat, actual_long))
                print("\tLocked index: %s" % (self.initial_index))
                print("\tDrift %d: (%0.6f, %0.6f)" % (len(self.calibration_positions), nth_drift))
                print("\tNew avg drift: (%0.6f, %0.6f)" % self.abs_drift)
                print("\tBearing: %0.6f" % bearing)
                print("----------")
            else:
            print("Invalid GPS. Ignoring")


    def brake_ping(self):
        self.brakes.ping()

    def steering_event(self):
        if self.steering.calibrated and self.manual_mode:
            # if self.joystick.get_axis("ZR") >= 1.0:
            joy_val = self.joystick.get_axis("right x")
            if abs(joy_val) > 0.0:
                offset = math.copysign(0.3, joy_val)
                joy_val -= offset

            delta_step = int(self.my_round(16 * self.sigmoid(10.0 * joy_val)))
            if abs(delta_step) > 0:
                self.steering.change_step(delta_step)

    @staticmethod
    def my_round(x, d=0):
        p = 10 ** d
        return float(math.floor((x * p) + math.copysign(0.5, x))) / p

    @staticmethod
    def sigmoid(x):  # modified sigmoid. -1...1
        return (-1 / (1 + math.exp(-x)) + 0.5) * 2

    def record_compass(self):
        if self.angle_filter.initialized:
            self.record("initial compass", self.angle_filter.compass_angle_packet)
            self.debug_print("initial offset: %0.4f rad" % self.angle_filter.compass_angle, ignore_flag=True)

    def close(self, reason):
        if reason != "done":
            self.brakes.pull()
            self.debug_print("!!EMERGENCY BRAKE!!", ignore_flag=True)
        else:
            self.brakes.pause_pings()
        self.quasar_plotter.close(reason)
        self.right_pipeline.close()
        print("Ran for %0.4fs" % self.dt())


class RoboQuasarPlotter:
    def __init__(self, animate, enable_plotting, enable_kalman, course_map, inner_map, outer_map, key_press_fn,
                 map_set_name):
        # GPS map based plots
        self.gps_plot = RobotPlot("gps", color="red", enabled=True)
        self.map_plot = RobotPlot("map", color="purple", marker='.')
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
        self.kalman_recomputed_plot = RobotPlot("kalman recomputed", color="green", enabled=self.enable_kalman)
        self.kalman_recorded_plot = RobotPlot("kalman recorded", color="gray", enabled=self.enable_kalman)

        self.steering_angle_plot = RobotPlot("steering angle")

        self.sticky_compass_counter = 0
        self.sticky_compass_skip = 100

        # plot collection
        self.accuracy_check_plot = RobotPlotCollection(
            "RoboQuasar plot",
            self.sticky_compass_plot, self.gps_plot,
            self.checkpoint_plot,
            self.map_plot, self.inner_map_plot, self.outer_map_plot,
            self.steering_plot, self.compass_plot_imu_only,
            self.compass_plot, self.goal_plot,
            self.current_pos_dot, self.kalman_recomputed_plot, self.kalman_recorded_plot,
            self.steering_angle_plot,
            window_resizing=False
        )

        if animate:
            self.plotter = LivePlotter(
                2, self.accuracy_check_plot,  # self.pipeline_plots,
                matplotlib_events=dict(key_press_event=key_press_fn), draw_legend=True,
                enabled=enable_plotting,
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

        for checkpoint_num in range(len(course_map)):
            self.plotter.draw_text(self.accuracy_check_plot, str(checkpoint_num),
                                   course_map.lats[checkpoint_num], course_map.longs[checkpoint_num], fontsize='small')

    def plot_image(self, map_set_name):
        if map_set_name in image_sets:
            image_info = image_sets[map_set_name]
        else:
            return

        # image_path, img_point_1, img_point_2, gps_point_1, gps_point_2 = image_info
        self.plotter.draw_image(self.accuracy_check_plot, *image_info)

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
        changed = False
        if outer_state and inner_state:
            self.position_state = "in bounds"
            status = 0
        elif not outer_state:
            self.position_state = "out of bounds! (outer)"
            status = 1
        elif not inner_state:
            self.position_state = "out of bounds! (inner)"
            status = 2

        if self.position_state != self.prev_pos_state:
            print("%0.4f: %s" % (dt, self.position_state))
            changed = True

            if status == 0:
                self.outer_map_plot.set_properties(color="blue")
                self.inner_map_plot.set_properties(color="blue")
            elif status == 1:
                self.outer_map_plot.set_properties(color="red")
            elif status == 2:
                self.inner_map_plot.set_properties(color="red")
        return status, changed

    def update_indicators(self, current_pos, imu_angle):
        if self.plotter.enabled:
            self.current_pos_dot.update([current_pos[0]], [current_pos[1]])
            if isinstance(self.plotter, LivePlotter):
                lat2, long2 = self.compass_coords(current_pos[0], current_pos[1], imu_angle)

                self.compass_plot.update([current_pos[0], lat2],
                                         [current_pos[1], long2])

    @staticmethod
    def compass_coords(lat, long, angle, length=0.0003):
        lat2 = length * math.cos(angle) + lat
        long2 = length * math.sin(angle) + long
        return lat2, long2

    def update_goal_angle(self, lat, long, goal_angle):
        goal_lat, goal_long = self.compass_coords(lat, long, goal_angle)
        self.steering_angle_plot.update([lat, goal_lat], [long, goal_long])

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
    "single" : (
        "Single Point",
        "Single Point Inside",
        "Single Point Outside",
        "single"
    ),
    "cut 3"  : (
        "Autonomous Map 3",
        "Autonomous Map 3 Inner",
        "Autonomous Map 3 Outer",
        "cut"
    ),
    "cut 2"  : (
        "Autonomous test map 2",
        "Autonomous test map 2 inside border",
        "Autonomous test map 2 outside border",
        "cut"
    ),
    "buggy"  : (
        "buggy course map",
        "buggy course map inside border",
        "buggy course map outside border",
        "buggy",
    ),
    "buggy 2": (
        "buggy course map 2",
        "buggy course map inside border 2",
        "buggy course map outside border 2",
        "buggy 2",
    ),
    "short"  : (
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
    "buggy"  : (
        "maps/buggy/Buggy Course Image 2.png",
        (427, 2040), (2611, 836),
        (40.43910438921088, -79.94750440120698),
        (40.44156221153447, -79.94164109230043),
    ),
    "buggy 2": (
        "maps/buggy/Buggy Course Image 2.png",
        (427, 2040), (2611, 836),
        (40.43910438921088, -79.94750440120698),
        (40.44156221153447, -79.94164109230043),
    ),
}

file_sets = {
    "rolls day 8"       : (
        ("05;37;41", "rolls/2017_Apr_02"),
        ("06;17;16", "rolls/2017_Apr_02"),
        ("06;23;15", "rolls/2017_Apr_02"),
    ),
    "data day 13"       : (
        ("17;23;27", "data_days/2017_Apr_01"),
        ("17;25;56", "data_days/2017_Apr_01"),
    ),
    "rolls day 7"       : (
        ("23;17", "rolls/2017_Mar_28"),
        ("23;20", "rolls/2017_Mar_28"),
        ("23;31", "rolls/2017_Mar_28"),
        ("23;41", "rolls/2017_Mar_28"),
        ("00;03", "rolls/2017_Mar_29"),
    ),
    "rolls day 6"       : (
        ("23;53", "rolls/2017_Mar_27"),
        ("00;44", "rolls/2017_Mar_28"),
        ("00;50", "rolls/2017_Mar_28"),
    ),
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
    "push practice 2": (
        ("00_10_35", "push_practice/2017_Mar_24", "mp4"),
        ("00_14_10", "push_practice/2017_Mar_24", "mp4"),
        ("00_17_45", "push_practice/2017_Mar_24", "mp4"),
        ("00_19_22", "push_practice/2017_Mar_24", "mp4"),
        ("00_38_15", "push_practice/2017_Mar_24", "mp4"),
        ("00_45_49", "push_practice/2017_Mar_24", "mp4"),
        ("00_47_21", "push_practice/2017_Mar_24", "mp4"),
        ("00_50_03", "push_practice/2017_Mar_24", "mp4"),
        ("00_51_05", "push_practice/2017_Mar_24", "mp4"),
    ),
    "data day 12"    : (
        ("16_36_17", "data_days/2017_Mar_18", "mp4"),
    ),
    "data day 11"    : (
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
