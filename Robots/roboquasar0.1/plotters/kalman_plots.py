from atlasbuggy.plotters.robotplot import RobotPlot, RobotPlotCollection


class KalmanPlots:
    def __init__(self, gps_enabled=True, kalman_enabled=True):
        self.gps_plot = RobotPlot(
            "GPS data", flat_plot=False, color='red', skip_count=0,
            # marker='.', linestyle='',
            plot_enabled=gps_enabled
        )

        self.kalman_plot = RobotPlot(
            "Kalman filter post", flat_plot=False, skip_count=0,
            # linestyle='', color='blue', marker='.',
            plot_enabled=kalman_enabled
        )

        self.compass_plot = RobotPlot("compass", color='orange', plot_enabled=kalman_enabled)
        self.filter_comparison = RobotPlotCollection("Comparison", self.kalman_plot, self.gps_plot, self.compass_plot,
                                                     flat_plot=True)

    def update_kalman(self, position_vector):
        self.kalman_plot.append(*position_vector)

    def update_gps(self, lat, long, alt):
        self.gps_plot.append(lat, long, alt)

    def update_compass(self, lat1, lat2, long1, long2):
        self.compass_plot.update([lat1, lat2], [long1, long2])
