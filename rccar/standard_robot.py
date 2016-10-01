import time
# from autobuggy import project
from autobuggy.robot import Robot
from autobuggy.microcontroller.logger import get_map
from joysticks.wiiu_joystick import WiiUJoystick
# from navigation.rccar_filter import RcCarFilter
from navigation.controller import Controller
from navigation.waypoint_picker import Waypoints
from standard_params import standard_params


class StandardRobot(Robot):
    def __init__(self, pipeline=None, capture=None, map_name=-1, map_dir=":gpx",
                 log_data=True, log_name=None,
                 log_dir=None):
        # set the project name (so that maps and logs and be found)
        # project.set_project_dir("rccar")

        self.manual_mode = True

        self.goal_x, self.goal_y = 0, 0
        self.controller = Controller()
        self.checkpoints = get_map("checkpoints.txt")
        self.checkpoint_num = 0
        self.waypoints = Waypoints(
            map_name, 1, map_dir
        )
        # filter = RcCarFilter(standard_params['counts_per_rotation'],
        #                      standard_params['wheel_radius'],
        #                      standard_params['front_back_dist'],
        #                      standard_params['max_speed'],
        #                      standard_params['left_angle_limit'],
        #                      standard_params['right_angle_limit'],
        #                      standard_params['left_servo_limit'],
        #                      standard_params['right_servo_limit'])
        filter = None

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
                         update_fn=None),#lambda: self.encoder_updated()),
            gps=dict(sensor_id=1, properties=['long', 'lat', 'fix'],
                     update_fn=None),#lambda: self.gps_updated()),
            imu=dict(sensor_id=2, properties=['yaw', 'accel_x', 'accel_y',
                                              'compass', 'ang_vx', 'ang_vy'],
                     update_fn=None),#lambda: self.yaw_updated()),
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
                        }),
            tilt_servo=dict(command_id=6, range=(-90, 90),
                        mapping={
                            "angle 1": -20,
                            "angle 2": 20
                        }),
            pan_servo=dict(command_id=7, range=(-90, 90),
                        mapping={
                            "angle 1": -20,
                            "angle 2": 20
                        }),
        )

        if log_dir is None:
            log_dir = ":today"  # today's date

        super(StandardRobot, self).__init__(
            sensors, commands, '/dev/ttyAMA0', None,
            filter, joystick, pipeline,
            capture, self.close_fn, log_data, log_name, log_dir)

        self.gps = self.sensors['gps']
        self.encoder = self.sensors['encoder']
        self.yaw = self.sensors['imu']

        self.servo = self.commands['servo']
        self.motors = self.commands['motors']
        self.blue_led = self.commands['blue_led']
        self.tilt_servo = self.commands['tilt_servo']
        self.pan_servo = self.commands['pan_servo']

    def button_dn(self, button, params):
        pass

    def axis_inactive(self, axis, params):
        if self.manual_mode:
            if axis == "left x":
                self.servo.set(0)
                # self.filter.update_servo(0)
            elif axis == "left y":
                self.blue_led.set(0)
                self.motors.set(0)
                # self.filter.update_motors(0)
            elif axis == "right x":
                self.pan_servo.set(0)
            elif axis == "right y":
                self.tilt_servo.set(0)


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
                # self.filter.update_servo(self.servo.get())
            elif axis == "left y":
                self.blue_led.set(int(abs(value) * 255))
                self.motors.set(int(-value * 100))
                # self.filter.update_motors(self.motors.get())
            elif axis == "right x":
                self.pan_servo.set(self.angle_to_servo(-value))
            elif axis == "right y":
                self.tilt_servo.set(self.angle_to_servo(value))

    def update_camera(self):
        if self.capture is not None:
            key = self.capture.key_pressed()

            if key == 'q' or key == "esc":
                print("quitting...")
                return False  # exit program
            elif key == ' ':
                if self.capture.paused:
                    print("%0.4fs: ...Video unpaused" % (
                        time.time() - self.time_start))
                else:
                    print("%0.4fs: Video paused..." % (
                        time.time() - self.time_start))
                    self.capture.paused = not self.capture.paused
            elif key == 's':
                self.capture.save_frame()
            elif key == 'v':
                if not self.capture.recording:
                    self.capture.start_recording()
                else:
                    self.capture.stop_recording()

            if not self.capture.paused:
                self.capture.show_frame()

        return True  # don't exit program

    def yaw_updated(self):
        self.filter.parse_imu(time.time() - self.time_start,
                              self.yaw.get('yaw'))

    def gps_updated(self):
        if self.gps.get("fix"):
            self.filter.parse_gps(time.time() - self.time_start,
                                  self.gps.get("long"),
                                  self.gps.get("lat"))
            if self.log_data:
                self.record("state", self.get_state())

    def encoder_updated(self):
        self.filter.parse_encoder(time.time() - self.time_start,
                                  self.encoder.get("counts"))
        if self.log_data:
            self.record("state", self.get_state())

    def close_fn(self):
        # stop the motors and reset the servo (they won't do it on their own!)
        self.motors.set(0)
        self.servo.set(0)

        self.blue_led.set(0)
