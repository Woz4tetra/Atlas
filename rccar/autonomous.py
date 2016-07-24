import math
import time
# from pprint import pprint

from standard_runner import StandardRunner


class Autonomous(StandardRunner):
    def __init__(self, use_initial_gps):
        super(Autonomous, self).__init__(log_data=True)

        if use_initial_gps:
            while not self.gps.get('fix'):
                print("waiting for fix...", self.gps)
                time.sleep(0.15)

            bearing = self.robot.filter.get_gps_bearing(
                -71.420864, 42.427317, -71.420795, 42.427332
            )
            bearing = (-bearing - math.pi / 2) % (2 * math.pi)
            initial_long = self.gps.get('long')
            initial_lat = self.gps.get('lat')
        else:
            initial_long, initial_lat = self.checkpoints[-1]
            second_long, second_lat = self.checkpoints[0]

            bearing = self.robot.filter.get_gps_bearing(
                initial_long, initial_lat, second_long, second_lat
            )
            bearing = (-bearing - math.pi / 2) % (2 * math.pi)

        self.robot.filter.initialize_filter(
            initial_long, initial_lat, bearing
        )
        self.robot.start()

    def main(self):
        state = self.robot.filter.state

        angle_command, speed_command = \
            self.controller.update(state, self.goal_x, self.goal_y)
        if 0.1 < speed_command < 0.6:
            speed_command = 0.6
        elif speed_command <= 0.1:
            speed_command = 0.0

        self.goal_x, self.goal_y = self.waypoints.get_goal(state)

        self.robot.record('controller', angle_command=angle_command,
                          speed_command=speed_command)
        self.robot.record('waypoints', goal_x=self.goal_x,
                          goal_y=self.goal_y)
        if not self.manual_mode:            
            self.servo.set(self.angle_to_servo(angle_command))
            self.motors.set(int(speed_command * 100))
            self.blue_led.set(int(speed_command * 255))


        time.sleep(0.1)


Autonomous(False).run()
