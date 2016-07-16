import sys

sys.path.insert(0, '../')

from manual.wiiu_joystick import WiiUJoystick
from robots.realbot import RealBot


class LoggerBot(RealBot):
    def __init__(self, initial_long=None, initial_lat=None,
                 initial_heading=0.0, log_data=True, log_dir=None,
                 enable_draw=True, enable_camera=True,
                 cam_width=320, cam_height=240):
        super(LoggerBot, self).__init__(
            initial_long, initial_lat, initial_heading, log_data, log_dir,
            enable_draw, enable_camera, cam_width, cam_height
        )

        # ----- Joystick -----
        self.joystick = WiiUJoystick(button_down_fn=self.button_dn,
                                     axis_active_fn=self.joy_changed,
                                     fn_params=self)

        self.joystick.start()

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

    @staticmethod
    def joy_changed(axis, value, self):
        if axis == "left x":
            self.servo.set(RealBot.stick_to_servo(value))
            self.filter.update_servo(self.servo.get())
        if axis == "left y":
            if value != 0:
                value = 1 * ((value < 0) - (value > 0))

            self.blue_led.set(int(value * 255 / 100))
            self.motors.set(int(value * 100))
            self.filter.update_motors(self.motors.get())

    def close(self):
        self.communicator.stop()
        self.capture.stop()
        self.joystick.stop()

        self.display_backlight(True)
