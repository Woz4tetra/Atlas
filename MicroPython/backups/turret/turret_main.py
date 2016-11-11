
from turret.turret_objects import *
from comm import Communicator

blue_led = BlueLEDcommand(1)

lidar = LidarSensor(0, uart_bus=1)

communicator = Communicator(blue_led, uart_bus=3)

while True:
#    communicator.read_command()

    if lidar.recved_data():
        communicator.write_packet(lidar)
        print(lidar)

#    pyb.delay(1)

    if communicator.should_reset():
        print("\nresetting")
        lidar.reset()
        
        communicator.write_packet(lidar)

    if communicator.should_stop():
        print("\nstopping")
        lidar.stop()

        while not communicator.should_reset():
            communicator.read_command()
            pyb.delay(5)
