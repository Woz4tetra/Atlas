
from quasar.quasar_objects import *
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
stepper = StepperCommand(4)

setting_up_sensors()

gps = GPS(1, uart_bus=2, timer_num=4)
imu = IMU(2, bus=1, reset_pin="Y7", timer_num=11)

communicator = Communicator(*leds, blue_led, stepper, uart_bus=4)

sensor_updated = False

comm_ready()

pyb.delay(500)
communicator.signal_stop()  # micropython will be in standby at the start
standby()

def reset():
    print("\nResetting sensors")
    gps.reset()
    imu.reset()
    stepper.reset()
    
    communicator.write_packet(gps)
    communicator.write_packet(imu)

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
       print("%8.5f (%s:%s:%s:%s), %7.4f, %7.4f, %7.4f; %10.6f, %10.6f, %10.6f        " % ((
           imu.data[0],) + imu.bno.get_calibration() + (imu.data[1], imu.data[2], imu.data[6], gps.data[0], gps.data[1], gps.data[2])), end='\r')
    pyb.delay(1)

    if communicator.should_stop():
        print("\nStopping sensors")
        gps.stop()
        imu.stop()

        standby()
        print("Entering standby")

        while not communicator.should_reset():
            communicator.read_command()
            pyb.delay(5)
        
        print("Exiting standby...")
        
        reset()

        comm_ready()
