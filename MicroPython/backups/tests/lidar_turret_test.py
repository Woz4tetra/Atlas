import pyb
from libraries.lidar_turret import LidarTurret

lidar = LidarTurret(1)

while True:
    if lidar.received():
        print(lidar.counts, "\t", lidar.distance)
    
