from pyb import Servo

print("tests:"
print("- servo")
print("- motor")
test_name = input("test to run:")
if test_name == "servo"
    servo = Servo(1)

    while True:
        value = input("> ")
        if value == 'r':
            servo.angle(-13)
        elif value == 'l':
            servo.angle(14)
        elif value == 'f':
            servo.angle(0)

        try:
            servo.angle(int(value))
        except ValueError:
            pass
elif test_name == "motor":
    pass
else:
    print("invalid test name")
