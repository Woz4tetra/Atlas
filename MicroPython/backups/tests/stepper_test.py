import pyb
from libraries.stepper import Stepper

stepper = Stepper(200, "X3", "X4", "X5", "X6")
stepper.set_speed(25)

while True:
    steps = int(input("steps: "))
    stepper.step(steps)
    pyb.delay(500)
