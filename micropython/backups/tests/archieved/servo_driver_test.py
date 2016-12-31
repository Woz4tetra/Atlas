import pyb
from libraries.pca9685 import ServoDriver

servo_driver = ServoDriver(2, -90, 90, 150, 600)

assert servo_driver.angle_to_pulse(-90) == 150
assert servo_driver.angle_to_pulse(90) == 600
assert servo_driver.angle_to_pulse(0) == 375

# servo_driver.servo_angle_min = 
# servo_driver.servo_angle_max = 
# servo_driver.servo_pulse_min = 
# servo_driver.servo_pulse_max = 
servo_driver.conversion = \
    (servo_driver.servo_pulse_max - servo_driver.servo_pulse_min) / (
        servo_driver.servo_angle_max - servo_driver.servo_angle_min)

for value in range(servo_driver.servo_angle_min,
                   servo_driver.servo_angle_max + 1, 10):
    for servo_num in range(16):
        servo_driver.set_servo(servo_num, value)
    print(value)
    pyb.delay(200)
