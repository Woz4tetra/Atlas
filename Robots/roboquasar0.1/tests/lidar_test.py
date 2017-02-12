from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from sensors.lidarturret import LidarTurret
from atlasbuggy.plotters.liveplotter import LivePlotter

from breezyslam.examples.pltslamshow import SlamShow

simulated = False
point_cloud_dir = "point_clouds/"


class LidarRunner(RobotInterface):
    def __init__(self):
        self.turret = LidarTurret()
        self.scan_file = open("scan_analyzers/scan.dat", mode="w+")

        self.use_my_plotter = True
        if self.use_my_plotter:
            self.live_plot = LivePlotter(
                2,
                self.turret.raw_point_cloud_plot,
                self.turret.point_cloud_plot,
                # self.turret.point_cloud_plot_collection,
                legend_args=dict(loc="upper left")
            )
            self.live_plot.draw_dot("point cloud", 0, 0, color='orange', markersize=5)
            self.live_plot.draw_dot("raw point cloud", 0, 0, color='orange', markersize=5)
        else:
            self.display = SlamShow(self.turret.slam.map_size_pixels, self.turret.slam.map_size_meters * 1000 / self.turret.slam.map_size_pixels, 'SLAM')

        super(LidarRunner, self).__init__(
            self.turret,
            log_data=False,
            # debug_prints=True,
            log_dir=(point_cloud_dir, None)
        )

    # def start(self):
    #     self.live_plot.start_time(self.start_time)

    def packet_received(self, timestamp, whoiam, packet):
        if self.did_receive(self.turret):
            if self.turret.did_cloud_update() and self.queue_len() < 25:  # and self.live_plot.should_update(timestamp):
                if self.turret.did_slam_update():
                    scan = "%s 0 0 " % int(self.dt * 1E6)
                    scan += "0 " * 22
                    scan += " ".join([str(x) for x in self.turret.distances])
                    scan += "\n"
                    print(scan)
                    self.scan_file.write(scan)

                if self.use_my_plotter:
                    if not self.live_plot.plot():
                        return False
                else:
                    self.display.displayMap(self.turret.slam.mapbytes)
                    self.display.setPose(*self.turret.slam.algorithm.getpos())
                    key = self.display.refresh()
                    if key is not None and (key & 0x1A):
                        return False

    def close(self):
        if self.use_my_plotter:
            self.live_plot.close()
        self.turret.slam.make_image()
        self.scan_file.close()


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret()

        self.animate = True
        if self.animate:
            self.animation = LivePlotter(1, self.turret.point_cloud_plot)
            self.animation.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarPlotter, self).__init__(None, point_cloud_dir + "2017_Feb_11", self.turret)

    def object_packet(self, timestamp):
        if self.did_receive(self.turret):
            if self.turret.did_cloud_update():
                if self.animate:
                    if not self.animation.plot():
                        return False
                else:
                    input("> ")

    def close(self):
        if self.animate:
            self.animation.close()


def run_lidar():
    if simulated:
        LidarPlotter().run()
    else:
        LidarRunner().run()


run_lidar()
