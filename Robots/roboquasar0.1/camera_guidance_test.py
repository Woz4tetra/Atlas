import math
from atlasbuggy.interface.live import RobotRunner

from joysticks.wiiu_joystick import WiiUJoystick

from actuators.brakes import Brakes
from actuators.steering import Steering
from actuators.underglow import Underglow
from sensors.gps import GPS
from sensors.imu import IMU

from atlasbuggy.vision.camera import Camera
from atlasbuggy.robot import Robot

from roboquasar import Pipeline


class PipelineRobot(Robot):
    def __init__(self):
        self.gps = GPS()
        self.imu = IMU()

        self.steering = Steering()
        self.brakes = Brakes()
        self.underglow = Underglow()

        self.left_camera = Camera("leftcam", enabled=True, show=False)
        self.right_camera = Camera("rightcam", enabled=False, show=False)
        self.left_pipeline = Pipeline(self.left_camera, separate_read_thread=False)
        self.right_pipeline = Pipeline(self.right_camera, separate_read_thread=False)

        self.manual_mode = True

        super(PipelineRobot, self).__init__(self.steering, self.brakes, self.underglow, self.imu, self.gps)

    def start(self):
        file_name = self.get_path_info("file name no extension").replace(";", "_")
        directory = self.get_path_info("input dir")
        self.open_cameras(file_name, directory, "avi")

        self.left_pipeline.start()
        self.right_pipeline.start()

    def open_cameras(self, log_name, directory, file_format):
        logitech_name = "%s%s.%s" % (log_name, self.left_camera.name, file_format)
        ps3eye_name = "%s%s.%s" % (log_name, self.right_camera.name, file_format)

        if self.logger is None:
            record = True
        else:
            record = self.logger.is_open()

        if self.is_live:
            status = self.left_camera.launch_camera(
                logitech_name, directory, record,
                capture_number=1
            )
            if status is not None:
                return status

            status = self.right_camera.launch_camera(
                ps3eye_name, directory, record,
                capture_number=2
            )
            if status is not None:
                return status
        else:
            self.left_camera.launch_video(logitech_name, directory, start_frame=0)
            self.right_camera.launch_video(ps3eye_name, directory)

    def loop(self):
        if not self.manual_mode:
            if self.left_pipeline.did_update():
                value = self.left_pipeline.safety_value
                if value > self.left_pipeline.safety_threshold:
                    self.steering.send_step(self.steering.left_limit * value)

            if self.right_pipeline.did_update():
                value = self.right_pipeline.safety_value
                if value > self.right_pipeline.safety_threshold:
                    self.steering.send_step(self.steering.right_limit * value)

            if self.left_pipeline.status is not None:
                return self.left_pipeline.status

            if self.right_pipeline.status is not None:
                return self.right_pipeline.status

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
                    if self.joystick.dpad[0] != 0:
                        self.steering.change_position(-self.joystick.dpad[0] * 10)

            if self.joystick.button_updated("X") and self.joystick.get_button("X"):
                self.manual_mode = not self.manual_mode
                print("Manual control enabled" if self.manual_mode else "Autonomous mode enabled")

            elif self.joystick.button_updated("L"):
                if self.joystick.get_button("L"):
                    self.brakes.unbrake()
                else:
                    self.brakes.brake()
            elif self.joystick.button_updated("R") and self.joystick.get_button("R"):
                self.brakes.toggle()

    def close(self, reason):
        self.left_camera.close()
        self.right_camera.close()

        self.left_pipeline.close()
        self.right_pipeline.close()
        print("Ran for %0.4fs" % self.dt())

robot = PipelineRobot()

log_dir = ("data_days", None)
runner = RobotRunner(robot, WiiUJoystick(), log_data=False, log_dir=log_dir, debug_prints=False)
runner.run()
