import pyb
from libraries.stepper import Stepper

stepper = Stepper(200, "X3", "X4", "X5", "X6")
stepper.set_speed(60)

while True:
    stepper.step(200)
    pyb.delay(500)
    print("200 steps")
    stepper.step(-200)
    pyb.delay(500)
    print("-200 steps")
