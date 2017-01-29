from pyb import Servo
from objects import RCmotors

print("tests:")
print("- (s)ervo")
print("- (m)otor")

servo = Servo(1)
motors = RCmotors(0)

motors.speed(0)

test_name = input("test to run: ")


if test_name == "s":
    while True:
        value = input("> ")
        if value == 'r':
            servo.angle(-12)
        elif value == 'l':
            servo.angle(14)
        elif value == 'f':
            servo.angle(2)
        else:
            try:
                servo.angle(int(value))
            except ValueError:
                pass
elif test_name == "m":
    while True:
        value = input("> ")
        if value == 'f':
            motors.speed(100)
        elif value == 'b':
            motors.speed(-100)
        elif value == 's':
            motors.speed(0)
        else:
            try:
                motors.speed(int(value))
            except ValueError:
                pass
else:
    print("invalid test name")
