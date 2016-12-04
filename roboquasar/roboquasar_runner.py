import time
import sys
from roboquasar_bot import RoboQuasarBot
from roboquasar_constants import constants
from autobuggy.filters.kalman_filter import GrovesKalmanFilter, \
    get_gps_orientation


class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data):
        super(RoboQuasarRunner, self).__init__(
            "track field checkpoints.gpx",
            "track field course map.gpx",
            log_data
        )
        self.checkpoint_num = 0

        lat1, long1 = self.checkpoints[0]
        lat2, long2 = self.checkpoints[1]
        altitude = 287

        initial_yaw, initial_pitch, initial_roll = \
            get_gps_orientation(lat1, long1, altitude, lat2, long2, altitude)

        self.filter = GrovesKalmanFilter(
            initial_roll=initial_roll,
            initial_pitch=initial_pitch,
            initial_yaw=initial_yaw,
            initial_lat=lat1,
            initial_long=long1,
            initial_alt=altitude,
            **constants
        )

        self.state = {
            "lat": lat1,
            "long": long1,
            "alt": altitude,
            "yaw": initial_yaw,
            "pitch": initial_pitch,
            "roll": initial_roll,
        }

        self.record("kalman", self.state)

        self.manual_mode = True

        self.gps_t0 = time.time()
        self.imu_t0 = time.time()

        self.steps_per_radian = 1.5
        self.current_step = 0
        self.goal_step = 0
        self.increment_value = 5

        self.accel_status_time0 = time.time()
        self.gyro_status_time0 = time.time()
        self.gps_status_time0 = time.time()

        self.start()

        while not self.sensor_status():
            self.print_sensors()
            time.sleep(0.05)

    def sensor_status(self):
        if (self.imu.get("ax") == 0.0 or
                    self.imu.get("ay") == 0.0 or
                    self.imu.get("az") == 0.0) and (
                    (time.time() - self.accel_status_time0) > 0.5):
            print("Accelerometer isn't responding!")
            return False
        if (self.imu.get("ax") != 0.0 or
                    self.imu.get("ay") != 0.0 or
                    self.imu.get("az") != 0.0):
            self.accel_status_time0 = time.time()

        if (self.imu.get("gx") == 0.0 or
                    self.imu.get("gy") == 0.0 or
                    self.imu.get("gz") == 0.0) and (
                    (time.time() - self.gyro_status_time0) > 0.5):
            print("Gyroscope isn't responding!")
            return False
        if (self.imu.get("gx") != 0.0 or
                    self.imu.get("gy") != 0.0 or
                    self.imu.get("gz") != 0.0):
            self.gyro_status_time0 = time.time()

        if (self.gps.get("lat") == 0.0 or
                    self.gps.get("long") == 0.0 or
                    self.gps.get("altitude") == 0.0) and (
                    (time.time() - self.gps_status_time0) > 0.5):
            print("GPS isn't responding!")
            return False
        if (self.gps.get("lat") != 0.0 or
                    self.gps.get("long") != 0.0 or
                    self.gps.get("altitude") != 0.0):
            self.gps_status_time0 = time.time()

        return True

    def button_dn(self, button):
        if button == 'A':
            self.record('checkpoint', checkpoint_num=self.checkpoint_num)
            print("Checkpoint %i recorded!" % self.checkpoint_num)

            self.checkpoint_num += 1
        elif button == 'B':
            self.leds['red'].set("toggle")

        elif button == 'X':
            self.manual_mode = not self.manual_mode
            print("Manual mode is", self.manual_mode)

    def record_state(self):
        position = self.filter.get_position()
        orientation = self.filter.get_orientation()

        self.state["lat"] = position[0]
        self.state["long"] = position[1]
        self.state["alt"] = position[2]

        self.state["yaw"] = orientation[0]
        self.state["pitch"] = orientation[1]
        self.state["roll"] = orientation[2]

        self.record("kalman", self.state)

    def imu_updated(self):
        if self.sensor_status():
            imu_dt = time.time() - self.imu_t0
            self.filter.imu_updated(
                imu_dt, self.imu.get("ax"), self.imu.get("ay"),
                self.imu.get("az"),
                self.imu.get("gx"), self.imu.get("gy"), self.imu.get("gz")
            )
            self.imu_t0 = time.time()

            self.record_state()

    def gps_updated(self):
        if self.sensor_status():
            gps_dt = time.time() - self.gps_t0
            self.filter.gps_updated(
                gps_dt, self.gps.get("lat"), self.gps.get("long"),
                self.gps.get("altitude")
            )
            self.gps_t0 = time.time()

            self.record_state()
            self.print_sensors()

    @staticmethod
    def print_data(x, y, z):
        print("[%3.4f, %3.4f, %3.4f]" % (x, y, z))

    def print_sensors(self):
        self.print_data(
            self.state["lat"], self.state["long"], self.state["altitude"])
        self.print_data(
            self.state["yaw"], self.state["pitch"], self.state["roll"])
        print()
        self.print_data(
            self.imu.get("ax"), self.imu.get("ay"), self.imu.get("ay"))
        self.print_data(
            self.imu.get("gx"), self.imu.get("gy"), self.imu.get("gy"))
        self.print_data(
            self.gps.get("lat"), self.gps.get("long"), self.gps.get("altitude"))
        print("\033[F" * 7)

    def axis_active(self, axis, value):
        if axis == "left x":
            self.blue_led.set(int(abs(value * 255)))

    def axis_inactive(self, axis):
        if axis == "left x":
            self.blue_led.set(0)

    def step(self):
        if self.manual_mode:
            value = self.joystick.get_axis("left x")
            if abs(value) > 0:
                self.stepper.set(int(-value * 4))
            time.sleep(0.04)
        else:
            steering_angle = self.controller.update(
                self.state["lat"], self.state["long"], self.state["yaw"]
            )
            self.goal_step = int(steering_angle * self.steps_per_radian)

            if self.current_step < self.goal_step:
                self.current_step += self.increment_value
                self.stepper.set(self.current_step)
            elif self.current_step > self.goal_step:
                self.current_step -= self.increment_value
                self.stepper.set(self.current_step)


log_data = True
if len(sys.argv) >= 2:
    log_data = bool(int(sys.argv[1]))

RoboQuasarRunner(log_data).run()
