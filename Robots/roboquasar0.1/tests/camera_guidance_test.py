import math
import numpy as np

from algorithms.bozo_controller import MapManipulator
from algorithms.bozo_filter import AngleManipulator
from algorithms.pipeline import Pipeline

from atlasbuggy.robot import Robot

from atlasbuggy.vision.camera import Camera

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow


class CameraGuidanceTest(Robot):
    def __init__(self, enable_cameras=True, show_cameras=False):

        # ----- initialize robot objects -----

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        super(CameraGuidanceTest, self).__init__(
            self.steering, self.brakes, self.underglow,
        )

        self.manual_mode = True

        # ----- init CV classes ------

        self.left_camera = Camera("leftcam", enabled=enable_cameras, show=show_cameras)
        self.right_camera = Camera("rightcam", enabled=False, show=show_cameras)
        self.left_pipeline = Pipeline(self.left_camera, separate_read_thread=False)
        self.right_pipeline = Pipeline(self.right_camera, separate_read_thread=False)

        # ----- init filters and controllers
        # position state message (in or out of map)
        self.position_state = ""
        self.prev_pos_state = self.position_state

        # init controller
        self.steering_angle = 0.0

        self.link_reoccuring(0.008, self.steering_event)

    def start(self):
        # extract camera file name from current log file name
        file_name = self.get_path_info("file name no extension").replace(";", "_")
        directory = self.get_path_info("input dir")

        # start cameras and pipelines
        self.open_cameras(file_name, directory, "mp4")
        self.left_pipeline.start()
        self.right_pipeline.start()

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

    def received(self, timestamp, whoiam, packet, packet_type):
        self.left_pipeline.update_time(self.dt())

    def loop(self):
        status = self.update_pipeline()
        if status is not None:
            return status
        self.update_joystick()

    def steering_event(self):
        self.brakes.ping()
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
                    # self.delay_function(1.5, self.dt(), self.underglow.rainbow_cycle)
                else:
                    self.brakes.pull()
                    self.underglow.signal_brake()
                    # self.delay_function(1.5, self.dt(), self.underglow.rainbow_cycle)

            elif self.joystick.button_updated("R") and self.joystick.get_button("R"):
                self.brakes.toggle()
                if self.brakes.engaged:
                    self.underglow.signal_brake()
                    # self.delay_function(1.5, self.dt(), self.underglow.rainbow_cycle)
                else:
                    self.underglow.signal_release()
                    # self.delay_function(1.5, self.dt(), self.underglow.rainbow_cycle)

    def update_pipeline(self):
        if not self.manual_mode:
            if self.left_pipeline.did_update():
                value = self.left_pipeline.safety_value
                if value > self.left_pipeline.safety_threshold:
                    new_angle = self.steering_angle + self.steering.left_limit_angle * value
                    print(new_angle)
                    self.steering.set_position(new_angle)
                    self.gps_imu_control_enabled = False
                else:
                    self.gps_imu_control_enabled = True

            if self.right_pipeline.did_update():
                value = self.right_pipeline.safety_value
                if value > self.right_pipeline.safety_threshold:
                    new_angle = self.steering_angle + self.steering.right_limit_angle * value
                    self.steering.set_position(new_angle)
                    self.gps_imu_control_enabled = False
                else:
                    self.gps_imu_control_enabled = True

            if self.left_pipeline.status is not None:
                return self.left_pipeline.status

            if self.left_pipeline.did_pause():
                self.toggle_pause()

            if self.right_pipeline.status is not None:
                return self.right_pipeline.status

    def close(self, reason):
        if reason != "done":
            self.brakes.pull()
            self.debug_print("!!EMERGENCY BRAKE!!", ignore_flag=True)

        self.left_pipeline.close()
        self.right_pipeline.close()

        print("Ran for %0.4fs" % self.dt())
