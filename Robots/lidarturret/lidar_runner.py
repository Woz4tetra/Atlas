from atlasbuggy.robot.interface import RobotInterface
from lidar.lidarturret import LidarTurret

# from lidar.lidarturret import LidarTurret

live_plotting = True
if live_plotting:
    from atlasbuggy.plotters.liveplotter import LivePlotter


class LidarRunner(RobotInterface):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)

        if live_plotting:
            self.live_plot = LivePlotter(
                2, self.turret.point_cloud_plot,
                legend_args=dict(loc="upper left")
            )
            self.live_plot.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarRunner, self).__init__(
            self.turret,
            # joystick=WiiUJoystick(),
            # log_data=False,
            debug_prints=True,
        )

    def start(self):
        if live_plotting:
            self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.turret):
            if live_plotting:
                if self.turret.did_cloud_update() and self.queue_len() < 25:  # and self.live_plot.should_update(timestamp):
                    if not self.live_plot.plot():
                        return False

    def loop(self):
        if self.joystick is not None:
            if self.joystick.button_updated('A'):
                # self.turret.set_paused()
                pass

    def close(self):
        if live_plotting:
            self.live_plot.close()


def run_lidar():
    LidarRunner().run()


run_lidar()
