import time
# from pprint import pprint

from standard_robot import StandardRobot


class Autonomous(StandardRobot):
    def __init__(self):
        super(Autonomous, self).__init__(map_name="test goal track.gpx",
                                         map_dir=":gpx", log_data=True)

        initial_long, initial_lat = self.checkpoints[1]
        second_long, second_lat = self.checkpoints[2]

        bearing = self.filter.get_gps_bearing(
            initial_long, initial_lat, second_long, second_lat
        )

        self.filter.initialize_filter(
            initial_long, initial_lat, bearing
        )
        self.record("initial conditions", initial_long=initial_long,
                    initial_lat=initial_lat, initial_heading=bearing)
        self.checkpoint_num = 1
        self.start()

    def button_dn(self, button, params):
        if button == 'B':
            self.manual_mode = not self.manual_mode
            print("Switching to",
                  "manual mode!" if self.manual_mode else "autonomous!")
            if self.manual_mode:
                self.motors.set(0)
                self.servo.set(0)
            else:
                self.motors.set(100)
        elif button == 'A':
            self.record('checkpoint', num=self.checkpoint_num)
            print("Checkpoint %i recorded!" % self.checkpoint_num)
            self.checkpoint_num += 1

    def main(self):
        if not self.manual_mode:
            angle_command = self.controller.update(
                self.filter.state, self.goal_x, self.goal_y)
            self.goal_x, self.goal_y = self.waypoints.get_goal(
                self.filter.state)

            self.servo.set(self.angle_to_servo(angle_command))

        time.sleep(0.1)


Autonomous().run()
