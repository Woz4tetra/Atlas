import pyb
from libraries.stepper import Stepper

stepper = Stepper(200, "Y3", "Y4", "Y5", "Y6")
stepper.set_speed(25)

while True:
    steps = int(input("steps: "))
    stepper.step(steps)
    pyb.delay(500)
