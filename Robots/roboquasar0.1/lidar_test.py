from atlasbuggy.interface import run, simulate
from atlasbuggy.robot import Robot
from atlasbuggy.plotters.liveplotter import LivePlotter

from sensors.lidarturret import LidarTurret
from algorithms.slam.lidar_slam import SLAM
from algorithms.slam.slam_plots import SlamPlots


class LidarTest(Robot):
    def __init__(self, enable_plotting=True):
        self.turret = LidarTurret()
        self.slam = SLAM(self.turret)

        super(LidarTest, self).__init__(self.turret)

        self.plotter = LivePlotter(
            1, self.turret.point_cloud_plot, enabled=enable_plotting
        )
        self.slam_plots = SlamPlots(self.slam.map_size_pixels, self.slam.map_size_meters, skip_count=0)
        self.lidar_t0 = 0.0

        self.link_object(self.turret, self.received_lidar)

    def received_lidar(self, timestamp, packet, packet_type):
        if self.turret.did_cloud_update():
            if self.turret.is_cloud_ready():
                self.slam.update(timestamp, self.turret.distances, [0, 0, self.dt() - self.lidar_t0])
                self.slam_plots.update(self.slam.mapbytes, self.slam.get_pos())

                self.lidar_t0 = self.dt()
            return self.plotter.plot()

    def close(self, reason):
        if reason == "done":
            self.plotter.freeze_plot()
        self.plotter.close()
        self.slam.make_image(self.logger.full_path + " post map")


test_bot = LidarTest()
run(test_bot, log_dir=("point_clouds", None), log_data=False, debug_prints=True)
# simulate("22", "point_clouds/2017_Mar_02", test_bot)
