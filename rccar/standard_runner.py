import time

from buggypi.robot import *
from buggypi.microcontroller.logger import get_map
from joysticks.wiiu_joystick import WiiUJoystick
from navigation.buggypi_filter import BuggyPiFilter
from navigation.controller import Controller
from navigation.waypoint_picker import Waypoints

standard_params = dict(
    counts_per_rotation=6,
    wheel_radius=0.097,
    front_back_dist=0.234,
    max_speed=1,  # 0.88

    # physical limit of the servo in radians
    left_angle_limit=0.81096,
    right_angle_limit=-0.53719,

    # physical limit of the servo in servo counts
    left_servo_limit=35,
    right_servo_limit=-25,

    # the servo value at which the robot can't drive forward because it's turned too much
    left_turning_limit=25,
    right_turning_limit=-15
)


class StandardRunner(RobotRunner):
    def __init__(self, pipeline=None, capture=None, map_name=-1, map_dir=None, log_data=True, log_name=None,
                 log_dir=None):
        self.robot_params = standard_params
        self.manual_mode = True

        self.goal_x, self.goal_y = 0, 0
        self.controller = Controller()
        self.checkpoints = get_map("checkpoints.txt")
        self.waypoints = Waypoints(
            map_name, 1, map_dir
        )
        filter = BuggyPiFilter(self.robot_params['counts_per_rotation'], self.robot_params['wheel_radius'],
                               self.robot_params['front_back_dist'],
                               self.robot_params['max_speed'],
                               self.robot_params['left_angle_limit'], self.robot_params['right_angle_limit'],
                               self.robot_params['left_servo_limit'], self.robot_params['right_servo_limit'])

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
                self.robot_params['left_servo_limit'], self.robot_params['right_servo_limit']),
                       mapping={
                           "left": self.robot_params['left_turning_limit'],
                           "right": self.robot_params['right_turning_limit'],
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

        robot = Robot(sensors, commands, filter, joystick, pipeline, capture,
                      self.close, log_data, log_name, log_dir)

        self.gps = robot.sensors['gps']
        self.encoder = robot.sensors['encoder']
        self.yaw = robot.sensors['imu']

        self.servo = robot.commands['servo']
        self.motors = robot.commands['motors']
        self.blue_led = robot.commands['blue_led']

        super(StandardRunner, self).__init__(robot)

    def button_dn(self, button, params):
        if button == 'B':
            self.manual_mode = not self.manual_mode
            print("Switching to",
                  "manual mode!" if self.manual_mode else "autonomous!")
            if not self.manual_mode:
                self.motors.set(100)

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
        return int(((self.robot_params['left_servo_limit '] - self.robot_params['right_servo_limit']) /
                    (self.robot_params['left_angle_limit '] - self.robot_params['right_angle_limit']) *
                    (angle - self.robot_params['right_angle_limit']) + self.robot_params['right_servo_limit']))

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
        self.robot.filter.update_imu(time.time() - self.robot.time_start,
                                     self.yaw.get('yaw'))

    def gps_updated(self):
        if self.gps.get("fix"):
            self.robot.filter.update_gps(time.time() - self.robot.time_start,
                                         self.gps.get("long"),
                                         self.gps.get("lat"))
            if self.robot.log_data:
                self.robot.record("state", self.robot.get_state())

    def encoder_updated(self):
        self.robot.filter.update_encoder(time.time() - self.robot.time_start,
                                         self.encoder.get("counts"))
        if self.robot.log_data:
            self.robot.record("state", self.robot.get_state())

    def close(self):
        self.blue_led.set(0)
