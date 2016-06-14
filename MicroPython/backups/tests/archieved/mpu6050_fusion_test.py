
import pyb

from libraries.mpu6050 import MPU6050
from libraries.hmc5883l import HMC5883L

from libraries.fusion import Fusion

imu = MPU6050(1, False)
compass = HMC5883L(1, declination=(-9, 16))

fuse = Fusion()

sw = pyb.Switch()
fuse.calibrate(compass.axes, sw, lambda : pyb.delay(100))
print(fuse.magbias)

count = 0
while True:
    fuse.update(imu.get_acc(), imu.get_gyro(), compass.axes())
    # if count % 50 == 0:
    print("Heading, Pitch, Roll: {:7.3f} {:7.3f} {:7.3f}".format(fuse.heading, fuse.pitch, fuse.roll))
    pyb.delay(20)
    count += 1
