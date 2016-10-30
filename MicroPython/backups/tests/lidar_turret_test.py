import pyb
from libraries.lidar_turret import LidarTurret

lidar = LidarTurret(2)

while True:
    if lidar.received():
        print(lidar.counts, "\t", lidar.rotations)
    
