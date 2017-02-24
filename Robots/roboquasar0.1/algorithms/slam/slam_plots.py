from algorithms.slam.pltslamshow import SlamShow


class SlamPlots:
    def __init__(self, map_size_pixels, map_size_meters, enabled=True, skip_count=0):
        self.skip_count = skip_count
        self.skip_counter = 0

        if enabled:
            self.slam_display = SlamShow(map_size_pixels,
                                         map_size_meters * 1000 / map_size_pixels, "SLAM")

            self.slam_display.refresh()

    def update(self, mapbytes, position_vector):
        if self.skip_count > 0:
            self.skip_counter += 1
            if self.skip_counter % self.skip_count != 0:
                return

        self.slam_display.displayMap(mapbytes)
        self.slam_display.setPose(*position_vector)

        self.slam_display.refresh()
