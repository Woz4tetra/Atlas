import os

from manual.wiiu_joystick import WiiUJoystick
from microcontroller.comm import *
from microcontroller.data import *
from microcontroller.logger import get_checkpoints
from navigation.buggypi_filter import BuggyPiFilter
from vision.camera import Camera


class BuggyPi:
    def __init__(self, initial_long=None, initial_lat=None, initial_heading=0.0,
                 manual_mode=True, enable_draw=True, log_data=True,
                 log_dir=None):
        # ----- BuggyPi properties -----
        self.log_data = log_data
        self.manual_mode = manual_mode
        self.enable_draw = enable_draw
        if log_dir is None:
            self.log_dir = self.log_folder()
        else:
            self.log_dir = log_dir

        # ----- Sensors -----
        self.encoder = Sensor(0, 'encoder', 'counts')
        self.gps = Sensor(1, 'gps', ['lat', 'long', 'altitude', 'found'])
        self.yaw = Sensor(2, 'imu', 'yaw')
        # self.altitude = Sensor(3, 'altitude', 'altitude')
        self.checkpoint_num = 0
        self.checkpoints = get_checkpoints()

        # ----- Communication instances -----
        self.sensor_pool = SensorPool(self.encoder, self.gps, self.yaw)
        self.communicator = Communicator(
            self.sensor_pool, address='/dev/ttyAMA0', log_data=log_data,
            log_dir=self.log_dir)

        if not self.communicator.initialized:
            raise Exception("Communicator not initialized...")

        # ----- Commands -----
        self.leds = [Command(command_id, "led " + str(command_id), (0, 2),
                             self.communicator) for command_id in range(4)]
        self.servo = Command(4, 'servo', (-90, 90), self.communicator)
        self.motors = Command(5, 'motors', (-100, 100), self.communicator)

        # ----- Joystick -----
        self.joystick = WiiUJoystick(button_down_fn=self.button_dn,
                                     axis_active_fn=self.joy_changed,
                                     fn_params=self)

        # ----- Camera -----
        self.capture = Camera(320, 240, enable_draw=self.enable_draw,
                              framerate=32)
        self.paused = False

        # ---- Kalman Filter -----
        check_long, check_lat = self.checkpoints[0]
        if initial_long is None:
            initial_long = check_long
        if initial_lat is None:
            initial_lat = check_lat

        self.filter = BuggyPiFilter(
            initial_long, initial_lat, initial_heading,
            counts_per_rotation=6, wheel_radius=0.097,
            front_back_dist=0.234, max_speed=0.88
        )

        # ----- Turn display off? -----
        if not self.enable_draw:
            print("Display will now turn off")
            time.sleep(2)
            os.system(
                "sudo echo 1 > /sys/class/backlight/rpi_backlight/bl_power")

        # ----- Start joystick and comm threads -----
        self.joystick.start()
        self.communicator.start()

        self.time_start = time.time()

    def update(self):
        self.update_filter()
        self.update_camera()
        # if not self.manual_mode:  # do path planning and command stuff

    def update_filter(self):
        if self.yaw.received():
            print(self.yaw)
            self.filter.update_imu(time.time() - self.time_start,
                                   -self.yaw.get('yaw'))
        if self.gps.received():
            print(self.gps)
            self.filter.update_gps(time.time() - self.time_start,
                                   self.gps.get("long"), self.gps.get("lat"))
        if self.encoder.received():
            print(self.encoder)
            self.filter.update_encoder(time.time() - self.time_start,
                                       self.encoder.get("counts"))

    def update_camera(self):
        if self.capture.get_frame() is None:
            return False
        key = self.capture.key_pressed()

        if key == 'q' or key == "esc":
            return False
        elif key == ' ':
            if self.paused:
                print("%0.4fs: ...Video unpaused" % (
                    time.time() - self.time_start))
            else:
                print("%0.4fs: Video paused..." % (
                    time.time() - self.time_start))
            self.paused = not self.paused
        elif key == 's':
            self.capture.save_frame()
        elif key == 'v':
            if not self.capture.recording:
                self.capture.start_recording()
            else:
                self.capture.stop_recording()

        if self.capture.recording:
            self.capture.record_frame()

        self.capture.show_frame()

        return True

    def close(self):
        self.joystick.stop()
        self.communicator.stop()
        self.capture.stop()

        if not self.enable_draw:
            os.system(
                "sudo echo 0 > /sys/class/backlight/rpi_backlight/bl_power")

    @staticmethod
    def log_folder():
        month = time.strftime("%b")
        day = time.strftime("%d")
        year = time.strftime("%Y")
        return "%s %s %s" % (month, day, year)

    @staticmethod
    def stick_to_servo(x):
        return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))

    @staticmethod
    def button_dn(button, self):
        if button == 'B':
            self.manual_mode = not self.manual_mode
            print("Switching to",
                  "manual mode!" if self.manual_mode else "autonomous!")
        if button == 'A':
            self.communicator.record('checkpoint', num=self.checkpoint_num,
                                     long=self.gps.get("long"),
                                     lat=self.gps.get("lat"))
            self.checkpoint_num += 1
            print(
                "--------\nCheckpoint reached! %s\n--------" % str(
                    self.checkpoint_num))

    @staticmethod
    def joy_changed(axis, value, self):
        if self.manual_mode:
            if axis == "left x":
                self.servo.set(BuggyPi.stick_to_servo(value))
                self.filter.update_servo(self.servo.get())
            if axis == "left y":
                if value != 0:
                    value = 1 * ((value > 0) - (value < 0))
                self.motors.set(int(value * 100))
                self.filter.update_motors(self.motors.get())
