import time
import sys
from roboquasar_bot import RoboQuasarBot
from roboquasar_constants import constants
from autobuggy.filters.kalman_filter import GrovesKalmanFilter, \
    get_gps_orientation


class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data):
        super(RoboQuasarRunner, self).__init__("track field checkpoints.gpx",
                                               "buggy course map.gpx",
                                               log_data)
        self.checkpoint_num = 0

        lat1, long1 = self.checkpoints[0]
        lat2, long2 = self.checkpoints[1]
        altitude = 270

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

    def imu_updated(self):
        imu_dt = time.time() - self.imu_t0
#        if not self.manual_mode:
        self.filter.imu_updated(
            imu_dt, self.imu.get("ax"), self.imu.get("ay"), self.imu.get("az"),
            self.imu.get("gx"), self.imu.get("gy"), self.imu.get("gz")
        )

        self.imu_t0 = time.time()

    def gps_updated(self):
        # print("yaw: %2.4f\n"
        #       "a: (%2.4f, %2.4f, %2.4f)\ng: (%2.4f, %2.4f, %2.4f)\n"
        #       "m: (%2.4f, %2.4f, %2.4f)\n"
        #       % self.imu.get(all=True))
        #
        # data = self.gps.get(all=True)
        # data = tuple(data[:3]) + (data[-1],)
        # print("long: %2.6f, lat: %2.6f, alt: %2.4f, fix: %s\n" % data)

#        if not self.manual_mode:
        gps_dt = time.time() - self.gps_t0
        self.filter.gps_updated(
            gps_dt, self.gps.get("lat"), self.gps.get("long"), self.gps.get("altitude")
        )
        self.gps_t0 = time.time()
    
        print(self.state)
        print()

    def main(self):
        if self.manual_mode:
            value = self.joystick.get_axis("left x")
            if abs(value) > 0:
                self.stepper.set(int(-value * 4))
                self.blue_led.set(int(abs(value * 255)))
#        else:
        position = self.filter.get_position()
        orientation = self.filter.get_orientation()

        self.state["lat"] = position[0]
        self.state["long"] = position[1]
        self.state["alt"] = position[2]

        self.state["yaw"] = orientation[0]
        self.state["pitch"] = orientation[1]
        self.state["roll"] = orientation[2]

        self.record("kalman", self.state)

            # TODO: get goal and set steering based on these values
        time.sleep(0.04)


log_data = True
if len(sys.argv) >= 2:
    log_data = bool(int(sys.argv[1]))

RoboQuasarRunner(log_data).run()
