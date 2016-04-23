import spi
import time
import RPi.GPIO as GPIO

GPIO.cleanup()
GPIO.setmode(GPIO.BCM)
print(spi.openSPI())

ce0 = 20
GPIO.setup(ce0, GPIO.OUT)
GPIO.output(ce0, 0)

try:
    while True:
        data = tuple([int(datum) for datum in input("> ").split(",")])
        GPIO.output(ce0, 1)
        print("response:", spi.transfer(data))
        GPIO.output(ce0, 0)
except KeyboardInterrupt:
    spi.closeSPI()
