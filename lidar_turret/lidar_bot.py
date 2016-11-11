import time

from autobuggy import project
from autobuggy.robot import Robot


class LidarBot(Robot):
    def __init__(self, log_data):
        project.set_project_dir("lidar_turret")

        sensors = dict(
            lidar=dict(sensor_id=1,
                     properties=['counts', 'rotations', 'distance'],
                     update_fn=lambda: self.lidar_updated()),
        )

        commands = dict(
            lidar_commands=dict(command_array={
                "resume": 0,
                "pause": 1,
                "direction": 2,
                "speed": 3,
            },
                range=(0, 255),
                mapping={
                    "forward": 0,
                    "backward": 1,
                    "min_speed": 1,
                    "max_speed": 255,
                }
            )
        )

        super(LidarBot, self).__init__(
            sensors, commands, '/dev/tty.usbserial',
            close_fn=self.close_fn, log_data=log_data, log_dir=":today"
        )

        self.lidar = self.sensors["lidar"]
        self.lidar_resume = self.commands["lidar_commands"]["resume"]
        self.lidar_pause = self.commands["lidar_commands"]["pause"]
        self.lidar_direction = self.commands["lidar_commands"]["direction"]
        self.lidar_speed = self.commands["lidar_commands"]["speed"]

    def lidar_updated(self):
        print("%5.0i, %5.0i, %5.0i" % self.lidar.get(all=True))

    def main(self):
        time.sleep(0.1)

lidar_robot = LidarBot(True)
lidar_robot.run()
