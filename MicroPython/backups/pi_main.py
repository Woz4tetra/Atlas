
from objects import *
from comm import Communicator
from libraries.rc_motors import RCmotors

for _ in range(15):
    pyb.LED(3).toggle()
    pyb.delay(50)
pyb.LED(3).on()

rc_motors = RCmotors("X8", 200, 320, 50)

leds = [LEDcommand(index, index + 1) for index in range(3)]
blue_led = BlueLEDcommand(3)
servo = ServoCommand(4, 1, start_pos=0)
motors = MotorCommand(5, rc_motors)
tilt_servo = ServoCommand(6, 2, start_pos=0)
pan_servo = ServoCommand(7, 3, start_pos=0)


encoder = RCencoder(0, rc_motors)
gps = GPS(1, 6, 4)
imu = IMU(2, 2, 11)

communicator = Communicator(*leds, blue_led, servo, motors, pan_servo, tilt_servo, uart_bus=1)

while True:
    communicator.read_command()
    
    if imu.recved_data():
        sensor_updated = True
        communicator.write_packet(imu)

    if gps.recved_data():
        sensor_updated = True
        communicator.write_packet(gps)
        pyb.LED(3).toggle()

#    if encoder.recved_data():
#        sensor_updated = True
#        communicator.write_packet(encoder)
#        pyb.LED(2).toggle()

#    if sensor_updated:
#        sensor_updated = False
#        print("%0.5f, %0.6f, %0.6f, %0.6i" % (
#            imu.data[0], gps.data[0], gps.data[1], encoder.data[0]), end='\r')

    if communicator.should_reset():
        gps.reset()
        imu.reset()
        encoder.reset()

        servo.reset()
        motors.reset()

        communicator.write_packet(gps)
        communicator.write_packet(imu)
        communicator.write_packet(encoder)
 
