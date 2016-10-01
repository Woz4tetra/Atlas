import time

from autobuggy.vision.rccamera import RcCamera
from pipeline import Pipeline
from standard_robot import StandardRobot


class RCcarRunner(StandardRobot):
    def __init__(self, log_data=True, enable_autonomous=True, enable_camera=True):
        self.enable_autonomous = enable_autonomous
        self.enable_camera = enable_camera
        
        if self.enable_camera:
            cam_width = 480
            cam_height = 320
            pipeline = Pipeline(cam_width, cam_height, False)
            capture = RcCamera(cam_width, cam_height,
                               update_fn=lambda params: self.update_camera())
        else:
            pipeline = None
            capture = None
            
        
        super(RCcarRunner, self).__init__(pipeline, capture,
                                          map_name="test goal track.gpx", log_data=log_data)
        
        if self.enable_autonomous:
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
        if button == 'B' and self.enable_autonomous:
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
        if not self.manual_mode and self.enable_autonomous:
            angle_command = self.controller.update(
                self.filter.state, self.goal_x, self.goal_y)
            self.goal_x, self.goal_y = self.waypoints.get_goal(
                self.filter.state)

            self.servo.set(self.angle_to_servo(angle_command))
        print("%4.0i\t%0.4f\t(%0.6f\t%0.6f)   " % (self.encoder.get("counts"), self.yaw.get("yaw"), self.gps.get("long"), self.gps.get("lat")), end='\r')

        time.sleep(0.1)


RCcarRunner(enable_autonomous=False, enable_camera=False, log_data=True).run()
