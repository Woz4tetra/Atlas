import os
import sys

sys.path.insert(0, '../')

from robots.robot import Robot
from microcontroller.comm import *
from microcontroller.data import *
from microcontroller.logger import get_map
from vision.camera import Camera


def log_folder():
    month = time.strftime("%b")
    day = time.strftime("%d")
    year = time.strftime("%Y")
    return "%s %s %s" % (month, day, year)


class RealBot(Robot):
    def __init__(self, initial_long=None, initial_lat=None,
                 initial_heading=0.0, log_data=True, log_dir=None,
                 enable_draw=True, enable_camera=True,
                 cam_width=320, cam_height=240):

        super(RealBot, self).__init__(
            initial_long, initial_lat, initial_heading
        )

        # ----- BuggyPi properties -----
        self.log_data = log_data
        self.enable_draw = enable_draw
        self.enable_camera = enable_camera

        if log_dir is None:
            self.log_dir = log_folder()
        else:
            self.log_dir = log_dir

        # ----- Sensors -----
        self.encoder = Sensor(0, 'encoder', 'counts')
        self.gps = Sensor(1, 'gps', ['long', 'lat'])
        self.yaw = Sensor(2, 'imu', 'yaw')
        # self.altitude = Sensor(3, 'altitude', 'altitude')
        self.checkpoint_num = 0
        self.checkpoints = get_map("checkpoints")

        # ----- Communication instances -----
        self.sensor_pool = SensorPool(self.encoder, self.gps, self.yaw)
        self.communicator = Communicator(
            self.sensor_pool, address='/dev/ttyAMA0', log_data=log_data,
            log_dir=self.log_dir)

        if not self.communicator.initialized:
            raise Exception("Communicator not initialized...")

        # ----- Commands -----
        self.leds = CommandArray({
            "red": 0,
            "yellow": 1,
            "green": 2,
        }, "led", (0, 2), self.communicator,
            mapping={
                "off": 0,
                "on": 1,
                "toggle": 2
            }
        )
        self.blue_led = Command(3, "led blue", (0, 255), self.communicator)
        self.servo = Command(4, 'servo', (-90, 90), self.communicator,
                             mapping={
                                 "left": self.left_servo_limit,
                                 "right": self.right_servo_limit,
                                 "forward": 0
                             })
        self.motors = Command(5, 'motors', (-100, 100), self.communicator,
                              mapping={
                                  "forward": 100,
                                  "backward": -100,
                                  "stop": 0
                              })

        # ----- Camera -----
        if self.enable_camera:
            self.capture = Camera(cam_width, cam_height,
                                  enable_draw=self.enable_draw,
                                  framerate=32)
            self.paused = False

        # ----- Turn display off? -----
        if not self.enable_draw:
            print("Display will now turn off")
            time.sleep(2)
            self.display_backlight(False)

        # ----- Start joystick and comm threads -----
        self.communicator.start()

        self.time_start = time.time()
        self.running = True

    def update(self):
        self.update_filter()
        if not self.update_camera():
            self.close()
            return False
        return True

    def update_filter(self):
        sensors_updated = False
        if self.yaw.received():
            # print(self.yaw)
            sensors_updated = True
            self.pi_filter.update_imu(time.time() - self.time_start,
                                      self.yaw.get('yaw'))
        if self.gps.received():
            # print(self.gps)
            sensors_updated = True
            self.pi_filter.update_gps(time.time() - self.time_start,
                                      self.gps.get("long"), self.gps.get("lat"))
        if self.encoder.received():
            # print(self.encoder)
            sensors_updated = True
            self.pi_filter.update_encoder(time.time() - self.time_start,
                                          self.encoder.get("counts"))

        if sensors_updated:
            timestamp = time.time() - self.time_start
            self.state = self.pi_filter.update_filter(timestamp)
            if self.log_data:
                self.communicator.record("state", timestamp=timestamp,
                                         **self.state)

        return sensors_updated

    def update_camera(self):
        if self.enable_camera:
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

        return True  # True == don't exit program

    def close(self):
        if self.running:
            self.motors.set(0)
            self.servo.set(0)

            time.sleep(0.1)

            self.communicator.stop()
            self.capture.stop()
            self.display_backlight(True)

            self.running = False

    def display_backlight(self, show):
        if not self.enable_draw:
            if show is True:
                os.system(
                    "sudo echo 0 > /sys/class/backlight/rpi_backlight/bl_power")
            else:
                os.system(
                    "sudo echo 1 > /sys/class/backlight/rpi_backlight/bl_power")

    @staticmethod
    def stick_to_servo(x):
        return int(-math.atan2(x, 1) * 180 / (math.pi * 1.85))
