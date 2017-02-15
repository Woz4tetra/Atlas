from algorithms.lidar_slam import SLAM
from atlasbuggy.plotters.liveplotter import LivePlotter
from atlasbuggy.robot.interface import RobotInterface
from atlasbuggy.robot.simulator import RobotInterfaceSimulator
from sensors.lidarturret import LidarTurret
from tests.scan_analyzers.pltslamshow import SlamShow

simulated = True
record_scan = True
use_my_plotter = True
point_cloud_dir = "point_clouds/"


class LidarRunner(RobotInterface):
    def __init__(self):
        self.turret = LidarTurret(angle_range=(210, 330), reverse_range=True)
        self.slam = SLAM(self.turret.point_cloud_size, self.turret.detection_angle_degrees)

        if record_scan:
            self.scan_file = open("scan_analyzers/scan.dat", mode="w+")

        if use_my_plotter:
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
            self.display = SlamShow(self.slam.map_size_pixels,
                                    self.slam.map_size_meters * 1000 / self.slam.map_size_pixels, 'SLAM')
            self.display.refresh()

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
                if self.turret.is_cloud_ready():
                    if record_scan:
                        scan = "%s 0 0 " % int(self.dt * 1E6)
                        scan += " ".join([str(x) for x in self.turret.distances])
                        scan += "\n"
                        print(scan)
                        self.scan_file.write(scan)
                    if not use_my_plotter:
                        self.display.displayMap(self.slam.mapbytes)
                        self.display.setPose(*self.slam.get_pos())
                        key = self.display.refresh()
                        if key is not None and (key & 0x1A):
                            return False

                if use_my_plotter:
                    if not self.live_plot.plot():
                        return False

    def close(self):
        if use_my_plotter:
            self.live_plot.close()
        self.slam.make_image()
        if record_scan:
            self.scan_file.close()


class LidarPlotter(RobotInterfaceSimulator):
    def __init__(self):
        self.turret = LidarTurret()

        self.animate = True
        if self.animate:
            self.animation = LivePlotter(1, self.turret.point_cloud_plot)
            self.animation.draw_dot("point cloud", 0, 0, color='orange', markersize=5)

        super(LidarPlotter, self).__init__("16;20", "2017_Feb_14", self.turret)

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
