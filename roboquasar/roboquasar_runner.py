import time
import sys
from roboquasar_bot import RoboQuasarBot


class RoboQuasarRunner(RoboQuasarBot):
    def __init__(self, log_data):
        super(RoboQuasarRunner, self).__init__("track field checkpoints.gpx",
                                               log_data=log_data)
        self.checkpoint_num = 0

        self.time0 = time.time()

    def button_dn(self, button):
        if button == 'A':
            self.record('checkpoint', checkpoint_num=self.checkpoint_num)
            print("Checkpoint %i recorded!" % self.checkpoint_num)

            self.checkpoint_num += 1
        elif button == 'B':
            self.leds['red'].set("toggle")
        

    def gps_updated(self):
        print("yaw: %2.4f\nax: %2.4f, ay: %2.4f, az: %2.4f\n"
              "gx: %2.4f, gy: %2.4f, gz: %2.4f\n"
              "mx: %2.4f, my: %2.4f, mz: %2.4f\n" % self.imu.get(all=True))

        data = self.gps.get(all=True)
        data = tuple(data[:3]) + (data[-1],)
        print("long: %2.6f, lat: %2.6f, alt: %2.4f, fix: %s\n" % data)

    def main(self):
        value = self.joystick.get_axis("left x")
        if abs(value) > 0:
            self.stepper.set(int(-value * 7))
            self.blue_led.set(int(abs(value * 255)))
        time.sleep(0.07)


log_data = True
if len(sys.argv) >= 2:
    log_data = bool(sys.argv[1])

RoboQuasarRunner(log_data).run()
