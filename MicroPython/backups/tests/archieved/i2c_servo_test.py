
import pyb
from libraries.pca9685 import PCA9685

servo_controller = PCA9685(2)

position = -90

while True:
    position += 10
    if position > 90:
        position = -90

    for servo_num in range(4):
        servo_controller.set_servo(servo_num, position)
        print("servo %i: %i" % (servo_num, position))
        pyb.delay(100)
    pyb.delay(1000)

    position += 10
    if position > 90:
        position = -90
    for servo_num in range(4):
        servo_controller.set_servo(servo_num, position)
        print("servo %i: %i" % (servo_num, position))

    pyb.delay(1000)
