import time
import sys
from roboquasar_bot import RoboQuasarBot
from roboquasar_constants import constants
from autobuggy.filters.kalman_filter import GrovesKalmanFilter


class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data):
        super(RoboQuasarRunner, self).__init__("buggy course checkpoints.gpx",
                                               "buggy course map.gpx",
                                               log_data)
        self.checkpoint_num = 0

        self.filter = None  #GrovesKalmanFilter(**constants)
        self.manual_mode = True

        self.gps_t0 = time.time()
        self.imu_t0 = time.time()
    
        self.max_imu = 0

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
        if not self.manual_mode:
            self.filter.imu_updated(
                imu_dt, self.imu["ax"], self.imu["ay"], self.imu["az"],
                self.imu["gx"], self.imu["gy"], self.imu["gz"]
            )
        
        if self.imu.get("ax") > self.max_imu:
            self.max_imu = self.imu.get("ax")
        if self.imu.get("ay") > self.max_imu:
            self.max_imu = self.imu.get("ay")
        if self.imu.get("az") > self.max_imu:
            self.max_imu = self.imu.get("az")
            
        self.imu_t0 = time.time()

    def gps_updated(self):
        print("yaw: %2.4f\n"
              "a: (%2.4f, %2.4f, %2.4f)\ng: (%2.4f, %2.4f, %2.4f)\n"
              "m: (%2.4f, %2.4f, %2.4f)\n"
               % self.imu.get(all=True))

        data = self.gps.get(all=True)
        data = tuple(data[:3]) + (data[-1],)
        print("long: %2.6f, lat: %2.6f, alt: %2.4f, fix: %s\n" % data)
        
        print("max imu:", self.max_imu)
        
        if not self.manual_mode:
            gps_dt = time.time() - self.gps_t0
            self.filter.gps_updated(
                gps_dt, self.gps["lat"], self.gps["long"], self.gps["altitude"]
            )
        self.gps_t0 = time.time()

    def main(self):
        if self.manual_mode:
            value = self.joystick.get_axis("left x")
            if abs(value) > 0:
                self.stepper.set(int(-value * 4))
                self.blue_led.set(int(abs(value * 255)))
            time.sleep(0.04)
        else:
            position = self.filter.get_position()
            orientation = self.filter.get_orientation()



            # TODO: get goal and set steering based on these values

log_data = True
if len(sys.argv) >= 2:
    log_data = bool(int(sys.argv[1]))

RoboQuasarRunner(log_data).run()
