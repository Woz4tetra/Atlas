from algorithms.slam.pltslamshow import SlamShow


class SlamPlots:
    def __init__(self, map_size_pixels, map_size_meters, enabled=True):
        if enabled:
            self.slam_display = SlamShow(map_size_pixels,
                                         map_size_meters * 1000 / map_size_pixels, "SLAM")

            self.slam_display.refresh()

    def update(self, mapbytes, position_vector):
        self.slam_display.displayMap(mapbytes)
        self.slam_display.setPose(*position_vector)

        self.slam_display.refresh()
