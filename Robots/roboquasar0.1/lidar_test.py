from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from sensors.lidarturret import LidarTurret
from atlasbuggy.plotters.liveplotter import LivePlotter

simulated = False
point_cloud_dir = "point_clouds/"


class LidarRunner(RobotInterface):
    def __init__(self):
        self.turret = LidarTurret(enable_slam=False)
        self.turret.cloud_time_plot.enabled = False

        self.live_plot = LivePlotter(
            2, self.turret.point_cloud_plot,
            legend_args=dict(loc="upper left")
        )
        self.live_plot.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarRunner, self).__init__(
            self.turret,
            log_data=False,
            debug_prints=True,
            log_dir=point_cloud_dir + ":today"
        )

    def start(self):
        self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.turret):
            if self.turret.did_cloud_update() and self.queue_len() < 25:  # and self.live_plot.should_update(timestamp):
                if not self.live_plot.plot():
                    return False

    def loop(self):
        if self.joystick is not None:
            if self.joystick.button_updated('A'):
                # self.turret.set_paused()
                pass

    def close(self):
        self.live_plot.close()


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret()

        self.animation = LivePlotter(1, self.turret.point_cloud_plot)
        self.animation.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarPlotter, self).__init__(-1, point_cloud_dir + ":-1", self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            if self.turret.did_cloud_update():
                if not self.animation.plot():
                    return False

    def close(self):
        self.animation.close()


def run_lidar():
    if simulated:
        LidarPlotter().run()
    else:
        LidarRunner().run()


run_lidar()