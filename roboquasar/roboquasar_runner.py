import time
import sys
import math
import os
from roboquasar_bot import RoboQuasarBot
from roboquasar_constants import constants
from atlasbuggy.filters.kalman_filter import GrovesKalmanFilter, \
    get_gps_orientation

initialize_with_checkpoints = True

with os.popen('stty size', 'r') as terminal_window:
    terminal_rows, terminal_cols = (int(x) for x in
                                    terminal_window.read().split())
print("Terminal size:", terminal_rows, terminal_cols)


class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data, map_set):
        if "track" == map_set:
            checkpoints_name = "track_field/track field checkpoints.gpx"
            map_name = "track_field/track field course map.gpx"
        elif "buggy" == map_set:
            checkpoints_name = "buggy_course/buggy course checkpoints.gpx"
            map_name = "buggy_course/buggy course map.gpx"
        elif "cut" == map_set:
            checkpoints_name = "cut/cut course checkpoints.gpx"
            map_name = "cut/cut course map 1.gpx"
        else:
            raise ValueError("map_set invalid:", map_set)

        super(RoboQuasarRunner, self).__init__(
            checkpoints_name,
            map_name,
            log_data
        )
        self.checkpoint_num = 0
        self.checkpoint_start_time = time.time()
        self.checkpoint_time = time.time()

        self.state = {
            "lat": 0.0,
            "long": 0.0,
            "alt": 0.0,
            "yaw": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
        }

        if initialize_with_checkpoints:
            self.init_with_checkpoints()

        self.manual_mode = True

        self.gps_t0 = time.time()
        self.imu_t0 = time.time()

        self.steps_per_radian = 1.5
        self.current_step = 0
        self.goal_step = 0
        self.increment_value = 5

        # ----- status checker variables -----

        self.accel_status_time0 = time.time()
        self.gyro_status_time0 = time.time()
        self.gps_status_time0 = time.time()

        self.accel_ok = False
        self.gyro_ok = False
        self.gps_ok = False

        self.prev_ax = 0.0
        self.prev_ay = 0.0
        self.prev_az = 0.0

        self.prev_gx = 0.0
        self.prev_gy = 0.0
        self.prev_gz = 0.0

        self.prev_lat = 0.0
        self.prev_long = 0.0
        self.prev_alt = 0.0

        self.prev_status_time = time.time()

    def init_with_checkpoints(self):
        lat1, long1 = self.checkpoints[0]
        lat2, long2 = self.checkpoints[1]
        altitude = 287

        initial_yaw, initial_pitch, initial_roll = \
            get_gps_orientation(lat1, long1, altitude, lat2, long2, altitude)

        self.init_filter(lat1, long1, altitude, initial_yaw, initial_pitch,
                         initial_roll)

    def init_with_gps(self):
        lat1, long1 = self.gps.get("lat"), self.gps.get("long")
        initial_yaw = math.radians(float(input("Enter heading in degrees: ")))

        altitude = 287

        self.init_filter(lat1, long1, altitude, initial_yaw, 0.0, 0.0)

    def init_filter(self, lat1, long1, altitude, initial_yaw, initial_pitch,
                    initial_roll):
        self.filter = GrovesKalmanFilter(
            initial_roll=initial_roll,
            initial_pitch=initial_pitch,
            initial_yaw=initial_yaw,
            initial_lat=lat1,
            initial_long=long1,
            initial_alt=altitude,
            **constants
        )

        self.state["lat"] = lat1
        self.state["long"] = long1
        self.state["alt"] = altitude
        self.state["yaw"] = initial_yaw
        self.state["pitch"] = initial_pitch
        self.state["roll"] = initial_roll

        print("Kalman filter initial conditions:           ")
        print("[%9.4f, %9.4f, %9.4f]                       " % (
            lat1, long1, altitude))
        print("[%9.4f, %9.4f, %9.4f]                       " % (
            initial_yaw, initial_pitch, initial_roll))

        self.record("kalman", self.state)

    def print_with_spaces(self, string):
        if len(string) < terminal_cols:
            string = string + " " * (terminal_cols - len(string))
        else:
            string = string[0:terminal_cols]
        print(string)

    def setup(self):
        print("Waiting for sensors...")
        time.sleep(1)
        while not self.sensors_ready():
            self.check_sensor_status()
            self.print_sensors()
            time.sleep(0.05)
        self.print_with_spaces("")
        self.print_with_spaces("Sensors ready!")

        if not initialize_with_checkpoints:
            self.init_with_gps()

        self.gps_t0 = time.time()
        self.imu_t0 = time.time()

    def sensors_ready(self):
        return self.accel_ok and self.gyro_ok and self.gps_ok

    def check_sensor(self, x, y, z, prev_x, prev_y, prev_z, time0):
        current_avg = abs((x + y + z) / 3)
        prev_avg = abs((prev_x + prev_y + prev_z) / 3)

        if (current_avg == prev_avg) or (current_avg == 0.0):
            if (time.time() - time0) > 1:
                return False

        return True

    def check_sensor_status(self):
        if not self.check_sensor(
                self.imu.get("ax"), self.imu.get("ay"), self.imu.get("az"),
                self.prev_ax, self.prev_ay, self.prev_az,
                self.accel_status_time0):
            self.accel_ok = False
        else:
            self.accel_status_time0 = time.time()
            self.accel_ok = True

        self.prev_ax = self.imu.get("ax")
        self.prev_ay = self.imu.get("ay")
        self.prev_az = self.imu.get("az")

        if not self.check_sensor(
                self.imu.get("gx"), self.imu.get("gy"), self.imu.get("gz"),
                self.prev_gx, self.prev_gy, self.prev_gz,
                self.gyro_status_time0):
            self.gyro_ok = False
            return False
        else:
            self.gyro_status_time0 = time.time()
            self.gyro_ok = True

        self.prev_gx = self.imu.get("gx")
        self.prev_gy = self.imu.get("gy")
        self.prev_gz = self.imu.get("gz")

        if not self.check_sensor(
                self.gps.get("lat"), self.gps.get("long"),
                self.gps.get("altitude"),
                self.prev_lat, self.prev_long, self.prev_alt,
                self.gps_status_time0):
            self.gps_ok = False
            return False
        else:
            self.gps_status_time0 = time.time()
            self.gps_ok = True

        self.prev_lat = self.gps.get("lat")
        self.prev_long = self.gps.get("long")
        self.prev_alt = self.gps.get("altitude")

    def button_dn(self, button):
        if button == 'A':
            # TODO: add disable/enable joystick callbacks
            if self.checkpoint_num < len(self.checkpoints):
                self.record('checkpoint', checkpoint_num=self.checkpoint_num)

                self.checkpoint_num += 1
                self.checkpoint_time = time.time()
        elif button == 'B':
            self.leds['red'].set("toggle")

        elif button == 'X':
            self.manual_mode = not self.manual_mode

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
        if self.sensors_ready():
            imu_dt = time.time() - self.imu_t0
            self.filter.imu_updated(
                imu_dt,
                self.imu.get("ax"), self.imu.get("ay"), self.imu.get("az"),
                self.imu.get("gx"), self.imu.get("gy"), self.imu.get("gz")
            )
            self.imu_t0 = time.time()

            self.record_state()

    def gps_updated(self):
        if self.sensors_ready():
            gps_dt = time.time() - self.gps_t0
            self.filter.gps_updated(
                gps_dt, self.gps.get("lat"), self.gps.get("long"),
                self.gps.get("altitude")
            )
            self.gps_t0 = time.time()

            self.record_state()

    def print_data(self, sensor_name, x, y, z, status):
        self.print_with_spaces("%s: [%9.4f, %9.4f, %9.4f], status: %i" % (
            sensor_name, x, y, z, status))

    def print_sensors(self):
        self.check_sensor_status()

        self.print_data(
            "position",
            self.state["lat"], self.state["long"], self.state["alt"],
            self.filter.is_active)
        self.print_data(
            "orientation",
            self.state["yaw"], self.state["pitch"], self.state["roll"],
            self.filter.is_active)
        print(" " * terminal_cols)
        self.print_data("accel",
                        self.imu.get("ax"), self.imu.get("ay"),
                        self.imu.get("az"), self.accel_ok)
        self.print_data("gyro ",
                        self.imu.get("gx"), self.imu.get("gy"),
                        self.imu.get("gz"), self.gyro_ok)
        self.print_data("gps  ",
                        self.gps.get("lat"), self.gps.get("long"),
                        self.gps.get("altitude"), self.gps_ok)
        print("checkpoint: %2.0d, %7.3f" % (
            self.checkpoint_num, self.checkpoint_time - self.checkpoint_start_time))
        print("manual mode: %s  " % self.manual_mode)
        print("stepper: %6.0d" % self.current_step)
        print("\033[F" * 10)

    def axis_active(self, axis, value):
        if axis == "left x":
            self.blue_led.set(int(abs(value * 255)))

    def axis_inactive(self, axis):
        if axis == "left x":
            self.blue_led.set(0)

    def close_fn(self):
        self.blue_led.set(0)
        print()

    def loop(self):
        if self.manual_mode:
            value = self.joystick.get_axis("left x")
            if abs(value) > 0:
                self.stepper.set(int(-value * self.increment_value))
                self.current_step += self.stepper.get() 
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

        if (time.time() - self.prev_status_time) > 0.25:
            self.print_sensors()
            self.prev_status_time = time.time()
        time.sleep(0.04)


def run():
    log_data = True
    map_set = "cut"
    
    if len(sys.argv) >= 2:
        log_data = bool(int(sys.argv[1]))
    if len(sys.argv) >= 3:
        map_set = sys.argv[2]

    RoboQuasarRunner(log_data, map_set).run()

run()
