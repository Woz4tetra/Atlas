import math
import time
# from pprint import pprint

from standard_runner import StandardRunner


def get_gps_bearing(long, lat, prev_long, prev_lat):
    long = math.radians(long)
    lat = math.radians(lat)
    prev_long = math.radians(prev_long)
    prev_lat = math.radians(prev_lat)

    y = math.sin(long - prev_long) * math.cos(lat)
    x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
        prev_lat) * math.cos(lat) *
         math.cos(long - prev_long))

    bearing = math.atan2(y, x)
    bearing = (-bearing - math.pi / 2) % (2 * math.pi)
    return bearing


class Autonomous(StandardRunner):
    def __init__(self):
        super(Autonomous, self).__init__(log_data=True)

        while not self.gps.get('fix'):
            print("waiting for fix...", self.gps)
            time.sleep(0.15)

        bearing = get_gps_bearing(
            -71.420864, 42.427317, -71.420795, 42.427332
        )
        self.robot.filter.initialize_filter(
            self.gps.get('long'), self.gps.get('lat'), bearing
        )

    def main(self):
        if not self.robot.manual_mode:
            angle_command, speed_command = \
                self.controller.update(self.robot.get_state(), self.goal_x,
                                       self.goal_y)
            if 0.1 < speed_command < 0.6:
                speed_command = 0.6
            elif speed_command <= 0.1:
                speed_command = 0.0

            print(self.robot.filter.angle_to_servo(angle_command),
                  speed_command)
            state = self.robot.filter.get_state()
            print(("%0.7f, " * 3) % (state["x"], state["y"], state["angle"]))
            print(("%0.7f, " * 3) % (self.goal_x, self.goal_y, angle_command))
            self.servo.set(self.robot.filter.angle_to_servo(angle_command))
            self.motors.set(int(speed_command * 100))
            self.blue_led.set(int(speed_command * 255))

            self.robot.record('controller', angle_command=angle_command,
                              speed_command=speed_command)

            time.sleep(0.05)


Autonomous().main()
