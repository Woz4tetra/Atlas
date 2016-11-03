import pyb
from libraries.bno055 import BNO055

imu = BNO055(1)

while True:
    print(imu.get_euler())
    pyb.delay(5)  # don't read data too fast
