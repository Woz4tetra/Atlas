from autobuggy.robot import *
from autobuggy.microcontroller.logger import get_map
from joysticks.wiiu_joystick import WiiUJoystick
from navigation.rccar_filter import RcCarFilter
from navigation.controller import Controller
from navigation.waypoint_picker import Waypoints
from standard_params import standard_params


class StandardRunner(RobotRunner):
    def __init__(self, pipeline=None, capture=None, map_name=-1, map_dir=None,
                 log_data=True, log_name=None,
                 log_dir=None):
        self.manual_mode = True

        self.goal_x, self.goal_y = 0, 0
        self.controller = Controller()
        self.checkpoints = get_map("checkpoints.txt")
        self.checkpoint_num = 0
        self.waypoints = Waypoints(
            map_name, 1, map_dir
        )
        filter = RcCarFilter(standard_params['counts_per_rotation'],
                             standard_params['wheel_radius'],
                             standard_params['front_back_dist'],
                             standard_params['max_speed'],
                             standard_params['left_angle_limit'],
                             standard_params['right_angle_limit'],
                             standard_params['left_servo_limit'],
                             standard_params['right_servo_limit'])

        joystick = WiiUJoystick(
            button_down_fn=lambda button, params: self.button_dn(
                button, params),
            axis_active_fn=lambda axis, value, params: self.axis_active(
                axis, value, params),
            axis_inactive_fn=lambda axis, params: self.axis_inactive(
                axis, params),
        )

        sensors = dict(
            encoder=dict(sensor_id=0, properties='counts',
                         update_fn=lambda: self.encoder_updated()),
            gps=dict(sensor_id=1, properties=['long', 'lat', 'fix'],
                     update_fn=lambda: self.gps_updated()),
            imu=dict(sensor_id=2, properties='yaw',
                     update_fn=lambda: self.yaw_updated()),
        )
        commands = dict(
            leds=dict(command_array={
                "red": 0,
                "yellow": 1,
                "green": 2,
            }, range=(0, 2),
                mapping={
                    "off": 0,
                    "on": 1,
                    "toggle": 2
                }),
            blue_led=dict(command_id=3, range=(0, 255)),
            servo=dict(command_id=4, range=(
                standard_params['left_servo_limit'],
                standard_params['right_servo_limit']),
                       mapping={
                           "left": standard_params['left_turning_limit'],
                           "right": standard_params['right_turning_limit'],
                           "forward": 0
                       }),
            motors=dict(command_id=5, range=(-100, 100),
                        mapping={
                            "forward": 100,
                            "backward": -100,
                            "stop": 0
                        })
        )

        if log_dir is None:
            log_dir = ":today"  # today's date

        robot = Robot(sensors, commands, "rccar", '/dev/ttyAMA0',
                      filter, joystick, pipeline,
                      capture, self.close, log_data, log_name, log_dir)

        self.gps = robot.sensors['gps']
        self.encoder = robot.sensors['encoder']
        self.yaw = robot.sensors['imu']

        self.servo = robot.commands['servo']
        self.motors = robot.commands['motors']
        self.blue_led = robot.commands['blue_led']

        super(StandardRunner, self).__init__(robot)

    def button_dn(self, button, params):
        pass

    def axis_inactive(self, axis, params):
        if self.manual_mode:
            if axis == "left x":
                self.servo.set(0)
                self.robot.filter.update_servo(0)
            elif axis == "left y":
                self.blue_led.set(0)
                self.motors.set(0)
                self.robot.filter.update_motors(0)

    def angle_to_servo(self, angle):
        return int(((standard_params['left_servo_limit'] - standard_params[
            'right_servo_limit']) /
                    (standard_params['left_angle_limit'] - standard_params[
                        'right_angle_limit']) *
                    (angle - standard_params['right_angle_limit']) +
                    standard_params['right_servo_limit']))

    def axis_active(self, axis, value, params):
        if self.manual_mode:
            if axis == "left x":
                self.servo.set(self.angle_to_servo(-value))
                self.robot.filter.update_servo(self.servo.get())
            elif axis == "left y":
                self.blue_led.set(int(abs(value) * 255))
                self.motors.set(int(-value * 100))
                self.robot.filter.update_motors(self.motors.get())

    def update_camera(self):
        if self.robot.capture is not None:
            key = self.robot.capture.key_pressed()

            if key == 'q' or key == "esc":
                print("quitting...")
                return False  # exit program
            elif key == ' ':
                if self.robot.capture.paused:
                    print("%0.4fs: ...Video unpaused" % (
                        time.time() - self.robot.time_start))
                else:
                    print("%0.4fs: Video paused..." % (
                        time.time() - self.robot.time_start))
                    self.robot.capture.paused = not self.robot.capture.paused
            elif key == 's':
                self.robot.capture.save_frame()
            elif key == 'v':
                if not self.robot.capture.recording:
                    self.robot.capture.start_recording()
                else:
                    self.robot.capture.stop_recording()

            if not self.robot.capture.paused:
                self.robot.capture.show_frame()

        return True  # don't exit program

    def yaw_updated(self):
        self.robot.filter.parse_imu(time.time() - self.robot.time_start,
                                    self.yaw.get('yaw'))

    def gps_updated(self):
        if self.gps.get("fix"):
            self.robot.filter.parse_gps(time.time() - self.robot.time_start,
                                        self.gps.get("long"),
                                        self.gps.get("lat"))
            if self.robot.log_data:
                self.robot.record("state", self.robot.get_state())

    def encoder_updated(self):
        self.robot.filter.parse_encoder(time.time() - self.robot.time_start,
                                        self.encoder.get("counts"))
        if self.robot.log_data:
            self.robot.record("state", self.robot.get_state())

    def close(self):
        self.blue_led.set(0)
