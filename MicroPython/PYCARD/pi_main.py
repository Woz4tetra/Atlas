# hi
from objects import *
from comm import Communicator
from libraries.rc_motors import RCmotors

for _ in range(15):
    pyb.LED(3).toggle()
    pyb.delay(50)
pyb.LED(3).on()

i2c_bus = 2

rc_motors = RCmotors("X8", 50, 900, 50)

leds = PybLEDs(0)
servo = ServoDriver(1, i2c_bus, 60)
motors = MotorCommand(2, rc_motors)

encoder = RCencoder(0, rc_motors)
gps = GPS(1, 6, 4)
imu = IMU(2, i2c_bus, 11)

communicator = Communicator(leds, servo, motors, uart_bus=1)

while True:
    communicator.read_command()

    sensor_updated = False
    
    if imu.recved_data():
        sensor_updated = True
        communicator.write_packet(imu)

    if gps.recved_data():
        sensor_updated = True
        communicator.write_packet(gps)

    if encoder.recved_data():
        sensor_updated = True
        communicator.write_packet(encoder)

    if sensor_updated:
        print("%0.5f, %0.6f, %0.6f, %i" % (
            imu.data[0], gps.data[0], gps.data[1], encoder.data[0]))

    if communicator.should_reset():
        gps.reset()
        imu.reset()
        encoder.reset()

        servo.reset()
        motors.reset()

        communicator.write_packet(gps)
        communicator.write_packet(imu)
        communicator.write_packet(encoder)
 
