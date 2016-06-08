from pyb import Servo

print("tests:")
print("- (s)ervo")
print("- (m)otor")
test_name = input("test to run:")
if test_name == "s":
    servo = Servo(3)

    while True:
        value = input("> ")
        if value == 'r':
            servo.angle(-12)
        elif value == 'l':
            servo.angle(14)
        elif value == 'f':
            servo.angle(2)

        try:
            servo.angle(int(value))
        except ValueError:
            pass
elif test_name == "m":
    pass
else:
    print("invalid test name")
