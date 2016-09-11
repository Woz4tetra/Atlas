import pyb
from libraries.lidar_lite_v2 import LIDARLite

lidar = LIDARLite(2)

while True:
    print(lidar.distance())
    pyb.delay(1)
