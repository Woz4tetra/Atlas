
from objects import *
from data import *
from comm import Communicator
from libraries.rc_motors import RCmotors

rc_motors = RCmotors("X8", 100, 800, 50)

leds = [LEDcommand(index, index + 1) for index in range(4)]
servo = ServoCommand(4, 1, start_pos=0)
motors = MotorCommand(5, rc_motors)

encoder = RCencoder(0, rc_motors)
gps = GPS(1, 6, "Y3")
imu = IMU(2, 1)

communicator = Communicator(*leds, servo, motors)

while True:
    communicator.write_packet(imu)
    pyb.delay(1)
    
    if gps.new_data:
        gps.stream_data()
        communicator.write_packet(gps)
        pyb.delay(1)
    
    if encoder.recved_data():
        communicator.write_packet(encoder)
        pyb.delay(1)

    communicator.read_command()

    if communicator.reset:
        gps.reset()
        imu.reset()
        encoder.reset()

        servo.reset()
        motors.reset()

        communicator.write_packet(gps)
        pyb.delay(1)

        communicator.write_packet(imu)
        pyb.delay(1)

        communicator.write_packet(encoder)
        pyb.delay(1)

        communicator.reset = False
 
