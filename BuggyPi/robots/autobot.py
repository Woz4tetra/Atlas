import sys
import time

sys.path.insert(0, '../')

from manual.wiiu_joystick import WiiUJoystick
from robots.realbot import RealBot
from microcontroller.logger import get_map
from navigation.waypoint_picker import Waypoints
from navigation.pd_controller import Controller


class AutoBot(RealBot):
    def __init__(self, initial_long=None, initial_lat=None,
                 initial_heading=0.0, log_data=True, log_dir=None,
                 manual_mode=True, enable_draw=True, enable_camera=True,
                 cam_width=320, cam_height=240):
        super(AutoBot, self).__init__(
            initial_long, initial_lat, initial_heading, log_data, log_dir,
            enable_draw, enable_camera, cam_width, cam_height
        )

        self.manual_mode = manual_mode

        # ----- Planners -----
        self.controller = Controller(
            0.5, self.front_back_dist, 1, 1, self.left_angle_limit,
            self.right_angle_limit, self.left_servo_limit,
            self.right_servo_limit
        )

        self.waypoints = Waypoints(
            get_map(0), self.left_angle_limit, self.right_angle_limit
        )

        self.goal_x, self.goal_y = self.waypoints.map[0]
        self.waypoint_num = 0

        # ----- Joystick -----
        self.joystick = WiiUJoystick(button_down_fn=self.button_dn,
                                     axis_active_fn=self.joy_changed,
                                     fn_params=self)

        self.joystick.start()

    def update(self):
        if self.update_filter() and not self.manual_mode:
            speed, servo_value = self.controller.update(
                self.state, self.goal_x, self.goal_y
            )
            print(speed, servo_value)
            self.motors.set(int(speed))
            self.servo.set(servo_value)
        if not self.update_camera():
            return False
        return True

    @staticmethod
    def button_dn(button, self):
        if button == 'B':
            self.manual_mode = not self.manual_mode
            print("Switching to",
                  "manual mode!" if self.manual_mode else "autonomous!")
        elif button == 'A':
            self.communicator.record('checkpoint', num=self.checkpoint_num,
                                     long=self.gps.get("long"),
                                     lat=self.gps.get("lat"))
            print(
                "--------\nCheckpoint reached! %s\n--------" % str(
                    self.checkpoint_num))
            self.checkpoint_num += 1
        elif button == 'X':
            self.goal_x, self.goal_y = self.waypoints.map[self.waypoint_num]
            self.waypoint_num += 1
            print("Setting goal to (%s, %s)" % (self.goal_x, self.goal_y))

    @staticmethod
    def joy_changed(axis, value, self):
        if self.manual_mode:
            if axis == "left x":
                self.servo.set(RealBot.stick_to_servo(value))
                self.pi_filter.update_servo(self.servo.get())
            if axis == "left y":
                if value != 0:
                    value = 1 * ((value < 0) - (value > 0))
                self.motors.set(int(value * 100))
                self.pi_filter.update_motors(self.motors.get())

    def close(self):
        if self.running:
            self.motors.set(0)
            time.sleep(0.005)
            self.servo.set(0)
            time.sleep(0.005)

            self.communicator.stop()
            self.capture.stop()
            self.joystick.stop()
            
            self.display_backlight(True)

            self.running = False
