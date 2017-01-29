import pyb
from libraries.bno055 import BNO055

bus = 2
print(bus)
imu = BNO055(bus)

while True:
    pyb.delay(5)  # don't read data too fast
    print(imu.get_euler())
    print(imu.get_lin_accel())
    print(imu.get_gyro())
