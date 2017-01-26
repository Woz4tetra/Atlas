import re

# line = "Cats are smarter than dogs"
#
# match = re.match( r'(.*) are (.*?) .*', line, re.M|re.I)
# print(match.group(1))
# print(match.group(2))


packet = "Time: 21:5:44.984	Date: 26/1/2017	Fix: 1 quality: 1	Location: 4026.5852N, 7956.5419W	Location degrees: 40.4431, -79.9424	Speed (knots): 0.11	Angle: 200.83	Altitude: 208.00	Satellites: 8"
pattern = re.compile("""
    [a-zA-Z]*:\s*
    (?P<hour>\d*):(?P<minute>\d*):(?P<second>\d*).(?P<millisecond>\d*)\s
    [a-zA-Z]*:\s(?P<month>\d*)/(?P<day>\d*)/(?P<year>\d*)\s
    [a-zA-Z]*:\s(?P<fix>\d*)\s[a-zA-Z]*:\s(?P<quality>\d*)\s
    [a-zA-Z]*:\s(?P<latitude>[-\d.]*)(?P<latdir>[a-zA-Z]*)[, ]*
    (?P<longitude>[-\d.]*)(?P<longdir>[a-zA-Z]*)\s
    [a-zA-Z() :]*(?P<latdeg>[-\d.]*)[, ]*
    (?P<longdeg>[-\d.]*)\s
    [a-zA-Z() :]*(?P<speedknots>[-\d.]*)\s
    [a-zA-Z() :]*(?P<angle>[-\d.]*)\s
    [a-zA-Z() :]*(?P<altitude>[-\d.]*)\s
    [a-zA-Z() :]*(?P<satellites>\d*)
 """, re.VERBOSE)
float_pattern = re.compile("^\d+?\.\d+?$")

match = pattern.match(packet)

print(match.group())
print(match.groups())

hour = int(match.group("hour"))
minute = int(match.group("minute"))
second = int(match.group("second"))
millisecond = int(match.group("millisecond"))
month = int(match.group("month"))
day = int(match.group("day"))
year = int(match.group("year"))
fix = int(match.group("fix"))
quality = int(match.group("quality"))
latitude = float(match.group("latitude"))
latdir = 1 if match.group("latdir") == "N" else -1
longitude = float(match.group("longitude"))
longdir = 1 if match.group("longdir") == "E" else -1
latdeg = float(match.group("latdeg"))
longdeg = float(match.group("longdeg"))
speed = float(match.group("speedknots"))
angle = float(match.group("angle"))
altitude = float(match.group("altitude"))
satellites = int(match.group("satellites"))

print(hour, minute, second, millisecond, month, day, year, fix, quality, latitude * latdir, longitude * longdir, latdeg, longdeg, speed, angle, altitude, satellites)
# parsed = []
# for s in match.groups():
#    if s.isdigit():
#        parsed.append(int(s))
#    if float_pattern.match(s) is not None:
#        parsed.append(float(s))
# print(parsed)
