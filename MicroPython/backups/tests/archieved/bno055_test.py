import pyb

from libraries.bno055 import BNO055

bno = BNO055(2)

while True:
    # to send to pc:
    print("lin accel:", bno.get_lin_accel())
    print("gyro:", bno.get_gyro())
    print("quat:", bno.get_quat())
    print("euler:", bno.get_euler())

    # ignorable:
    print("temp:", bno.get_temp())
    print("accel:", bno.get_accel())
    print("grav:", bno.get_grav())
    print("mag:", bno.get_mag())

    pyb.delay(bno.sample_delay)
