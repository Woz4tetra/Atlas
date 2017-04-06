import re
from atlasbuggy.files.atlasbuggyfile import AtlasReadFile, AtlasWriteFile


class MapFile(AtlasReadFile):
    def __init__(self, map_name, map_dir=None):
        super(MapFile, self).__init__(map_name, map_dir, False, "gpx", "maps")

        self.open()

        self.map, self.lats, self.longs = self.parse_map()

    def parse_map(self):
        gps_map = []
        lats = []
        longs = []

        regex = r" lat=\"(?P<lat>[-0-9.]*)\" lon=\"(?P<lon>[-0-9.]*)\""

        matches = re.finditer(regex, self.contents)

        for match_num, match in enumerate(matches):
            lat = float(match.group("lat"))
            long = float(match.group("lon"))

            gps_map.append((lat, long))

            lats.append(lat)
            longs.append(long)

        return gps_map, lats, longs

    def __getitem__(self, item):
        return self.map[item]

    def __len__(self):
        return len(self.map)


class MapMaker(AtlasWriteFile):
    def __init__(self, map_name, map_dir=None):
        super(MapMaker, self).__init__(map_name, map_dir, False, "gpx", "maps")

        self.map_header = ("<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"no\" ?>\n"
                           "<gpx creator=\"Atlasbuggy GPX maker\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xmlns=\"http://www.topografix.com/GPX/1/1\" version=\"1.1\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/1 http://www.topografix.com/GPX/1/1/gpx.xsd\">\n"
                           "<metadata>\n"
                           "  <name>{name}</name>\n"
                           "  <desc></desc>\n"
                           "  <author><name>Atlasbuggy GPX maker</name></author>\n"
                           "<bounds minlat=\"{minlat}\" minlon=\"{minlon}\" maxlat=\"{maxlat}\" maxlon=\"{maxlon}\"/></metadata>\n"
                           "<trk><name>{name}</name><trkseg>\n"
                           )
        self.map_contents_format = "  <trkpt lat=\"{lat}\" lon=\"{lon}\"></trkpt>\n"
        self.map_end_format = "</trkseg></trk></gpx>\n"

        self.min_lat = None
        self.max_lat = None
        self.min_lon = None
        self.max_lon = None

        self.map = []

        self.open()

    def append(self, lat, lon):
        if self.min_lat is None or lat < self.min_lat:
            self.min_lat = lat
        if self.max_lat is None or lat > self.max_lat:
            self.max_lat = lat

        if self.min_lon is None or lon < self.min_lon:
            self.min_lon = lon
        if self.max_lon is None or lon > self.max_lon:
            self.max_lon = lon

        self.map.append((lat, lon))

    def extend(self, arg):
        if isinstance(arg, MapFile):
            gps_map = arg.map
        else:
            gps_map = arg
        for row in gps_map:
            self.append(row[0], row[1])

    def make_map(self):
        if len(self.map) > 0:
            self.write(self.map_header.format(name=self.file_name,
                                              minlat=self.min_lat, maxlat=self.max_lat, minlon=self.min_lon,
                                              maxlon=self.max_lon))
            for row in self.map:
                self.write(self.map_contents_format.format(lat=row[0], lon=row[1]))
            self.write(self.map_end_format)

            self.close()
        else:
            raise ValueError("No values appended to the map!")
