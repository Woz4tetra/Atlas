import pyb

from libraries.mpu6050 import MPU6050
from libraries.hmc5883l import HMC5883L

imu = MPU6050(1,False)
compass = HMC5883L(1, declination = (-9,16))

while True:
    accel_buf = imu.get_accel_raw()
    gyro_buf = imu.get_gyro_raw()
    axes = compass.heading()

    gyro_x = gyro_buf[0] << 8 | gyro_buf[1]
    gyro_y = gyro_buf[2] << 8 | gyro_buf[3]
    gyro_z = gyro_buf[4] << 8 | gyro_buf[5]
    if gyro_x >> 15 == 1:
        gyro_x -= 2 << 15
    if gyro_y >> 15 == 1:
        gyro_y -= 2 << 15
    if gyro_z >> 15 == 1:
        gyro_z -= 2 << 15

    accel_x = accel_buf[0] << 8 | accel_buf[1]
    accel_y = accel_buf[2] << 8 | accel_buf[3]
    accel_z = accel_buf[4] << 8 | accel_buf[5]
    if accel_x >> 15 == 1:
        accel_x -= 2 << 15
    if accel_y >> 15 == 1:
        accel_y -= 2 << 15
    if accel_z >> 15 == 1:
        accel_z -= 2 << 15


    #print((accel_x * 9.81/7900, accel_y * 9.81/7900, accel_z * 9.81/7900), "accel")
    print((gyro_x / 7150, gyro_y / 7150, gyro_z / 7150), "gyro")
    #print(axes, "axes")

    pyb.delay(200)
