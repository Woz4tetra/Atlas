    
from objects import *
from comm import Communicator

pyb_leds = [pyb.LED(num) for num in range(1, 4)]

def setting_up_sensors():
    pyb_leds[0].on()
    pyb_leds[1].off()
    pyb_leds[2].off()

def comm_ready():
    pyb_leds[0].off()
    pyb_leds[1].on()
    pyb_leds[2].off()

def gps_received():
    pyb_leds[1].toggle()

def standby():
    pyb_leds[0].off()
    pyb_leds[1].off()
    pyb_leds[2].on()

leds = [LEDcommand(index, index + 1) for index in range(3)]  # 3 normal LEDs
blue_led = BlueLEDcommand(3)
stepper = StepperCommand(4, ["X3", "X4", "X5", "X6"])

setting_up_sensors()

gps = GPS(1, uart_bus=1, timer_num=4)
imu = IMU(2, bus=2, reset_pin="X7", timer_num=11)

communicator = Communicator(*leds, blue_led, stepper, uart_bus=4)

sensor_updated = False

comm_ready()

while True:
    communicator.read_command()

    if imu.recved_data():
        sensor_updated = True
        communicator.write_packet(imu)

    if gps.recved_data():
        sensor_updated = True
        communicator.write_packet(gps)
        gps_received()

    if sensor_updated:
       sensor_updated = False
       print("%0.5f (%s:%s:%s:%s), %0.6f, %0.6f" % (
           imu.data[0],) + imu.bno.get_calibration() + (gps.data[0], gps.data[1]), end='\r')

    pyb.delay(1)

    if communicator.should_reset():
        print("\nresetting")
        gps.reset()
        imu.reset()

        communicator.write_packet(gps)
        communicator.write_packet(imu)
    
    if communicator.should_stop():
        print("\nstopping")
        gps.stop()
        imu.stop()
        
        standby()
        
        while not communicator.should_reset():
            communicator.read_command()
            pyb.delay(5)
        
        comm_ready()
