from atlasbuggy.files.atlasbuggyfile import AtlasReadFile


class MapFile(AtlasReadFile):
    def __init__(self, map_name, map_dir=None):
        super(MapFile, self).__init__(map_name, map_dir, False, "gpx", "maps")
        self.map = self.parse_map()

    def parse_map(self):
        gps_map = []
        # xml parsing. Extract the long and lat from the file
        start_flags = ['<rtept', '<trkpt']
        data_start = None
        for flag in start_flags:
            if self.contents.find(flag) != -1:
                data_start = flag
                break
        if data_start is None:
            raise ValueError("Invalid file format! Start flag not found...")

        start_index = self.contents.find(data_start) + len(data_start)
        for line in self.contents[start_index:].splitlines():
            line = line.strip(" ")
            unparsed = line.split(" ")

            if len(unparsed) > 1:
                if len(unparsed) == 2:
                    lat_unparsed, long_unparsed = unparsed
                else:
                    _, lat_unparsed, long_unparsed = unparsed

                lat = float(lat_unparsed[5:-1])
                long = float(long_unparsed[5:-10])

                gps_map.append((lat, long))
        return gps_map