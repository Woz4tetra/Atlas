import os

os.chdir("..")

from atlasbuggy.interface.live import RobotRunner
from atlasbuggy.plotters.liveplotter import LivePlotter

from sensors.lidarturret import LidarTurret
from algorithms.slam.lidar_slam import SLAM
from algorithms.slam.slam_plots import SlamPlots


class LidarTest(RobotRunner):
    def __init__(self):
        self.turret = LidarTurret()

        # self.turret.point_cloud_plot.enabled = False

        self.plotter = LivePlotter(1, self.turret.point_cloud_plot)

        super(LidarTest, self).__init__(self.turret, log_dir=("point_clouds", None))

        self.slam = SLAM(self.turret, self.logger.full_path + " post map")
        self.slam_plots = SlamPlots(self.slam.map_size_pixels, self.slam.map_size_meters, skip_count=0)
        self.lidar_t0 = 0

    def received(self, timestamp, whoiam, packet, packet_type):
        if self.did_receive(self.turret):
            if self.update_turret(timestamp) is False:
                return False

    def update_turret(self, timestamp):
        if self.turret.did_cloud_update():
            if self.turret.is_cloud_ready():
                self.slam.update(timestamp, self.turret.distances, [0, 0, self.dt() - self.lidar_t0])
                self.slam_plots.update(self.slam.mapbytes, self.slam.get_pos())

                print(self.slam.get_pos())

                self.lidar_t0 = self.dt()

            if not self.plotter.plot():
                return False

    def start(self):
        self.plotter.start_time(self.start_time)

    def close(self, reason):
        if reason == "done":
            print("Finished!")
            self.plotter.freeze_plot()

        self.plotter.close()
        self.slam.make_image()


LidarTest().run()
