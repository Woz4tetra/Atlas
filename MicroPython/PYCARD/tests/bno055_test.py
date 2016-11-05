import pyb
from libraries.bno055 import BNO055

imu = BNO055(2)

while True:
    pyb.delay(5)  # don't read data too fast
    print(imu.get_euler())
