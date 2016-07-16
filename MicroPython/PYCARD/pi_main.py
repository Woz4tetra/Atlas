
from objects import *
from comm import Communicator
from libraries.rc_motors import RCmotors

for _ in range(50):
    pyb.LED(3)
    pyb.delay(50)

rc_motors = RCmotors("X8", 50, 900, 50)

leds = [LEDcommand(index, index + 1) for index in range(3)]
blue_led = BlueLEDcommand(3)
servo = ServoCommand(4, 1, start_pos=0)
motors = MotorCommand(5, rc_motors)

encoder = RCencoder(0, rc_motors)
gps = GPS(1, 6, 4)
imu = IMU(2, 2, 11)

communicator = Communicator(*leds, servo, motors, uart_bus=1)

while True:
    communicator.read_command()

    if imu.recved_data():
        communicator.write_packet(imu)

    if gps.recved_data():
        communicator.write_packet(gps)

    if encoder.recved_data():
        communicator.write_packet(encoder)

    if communicator.should_reset():
        gps.reset()
        imu.reset()
        encoder.reset()

        servo.reset()
        motors.reset()

        communicator.write_packet(gps)
        communicator.write_packet(imu)
        communicator.write_packet(encoder)
 
