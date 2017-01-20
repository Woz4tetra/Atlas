from atlasbuggy.robot.interface import RobotInterface
from joysticks.wiiu_joystick import WiiUJoystick
from lidarturret import LidarTurret

# from atlasbuggy.plotters.robotplot import RobotPlot

live_plotting = True
if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter


class LidarRunner(RobotInterface):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)

        if live_plotting:
            self.live_plot = LivePlotter(
                1, self.turret.point_cloud_plot,
                legend_args=dict(loc="upper left")
            )

        super(LidarRunner, self).__init__(
            self.turret,
            # joystick=WiiUJoystick(),
            # log_data=False,
            # debug_prints=True,
        )

    def start(self):
        if live_plotting:
            self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.turret):
            # self.turret.update_slam(timestamp)
            # print(self.turret.distance)
            if live_plotting:
                if self.turret.did_cloud_update() and self.queue_len() < 10:  # and self.live_plot.should_update(timestamp):
                    if not self.live_plot.plot():
                        return False

    def loop(self):
        if self.joystick is not None:
            if self.joystick.button_updated('A'):
                self.turret.set_paused()

    def close(self):
        if self.turret.enable_slam:
            self.turret.slam.make_image("something")


def run_lidar():
    LidarRunner().run()


run_lidar()
