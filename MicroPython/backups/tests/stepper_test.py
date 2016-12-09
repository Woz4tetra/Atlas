import pyb
from libraries.stepper import Stepper

stepper = Stepper(200, 25, "Y3", "Y4", "Y5", "Y6")

while True:
    steps = int(input("steps: "))
    stepper.step(steps)
    pyb.delay(500)
