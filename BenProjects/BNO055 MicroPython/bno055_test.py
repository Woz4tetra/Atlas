import pyb

from libraries.bno055 import BNO055

bno = BNO055(1)

while true:
    # to send to pc:
    print(bno.get_lin_accel())
    print(bno.get_gyro())
    print(bno.get_quat())
    print(bno.get_euler())

    # ignorable:
    print(bno.get_temp())
    print(bno.get_accel())
    print(bno.get_grav())
    print(bno.get_mag())

    pyb.delay(bno.sample_delay)
