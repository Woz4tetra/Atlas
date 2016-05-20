import spidev
import time
from RPi import GPIO
import random

pin = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)

spi = spidev.SpiDev()
spi.open(0, 0)

GPIO.output(pin, False)

def command(*data):
    try:
        assert(len(data) == 5 or len(data) == 0)
    except AssertionError:
        print("data must be length 5")
        GPIO.cleanup()
    GPIO.output(pin, True)
    if len(data) == 0:
        start = random.randint(0, 250)
        data = [start + x for x in range(5)]
        print(data)
    spi.xfer2([1] + list(data))
    GPIO.output(pin, False)

def sensors():
    GPIO.output(pin, True)
    print(spi.writebytes([2, 0, 0, 0, 0, 0]))
    print(spi.readbytes(6))
    GPIO.output(pin, False)

def end():
    GPIO.output(pin, False)
    spi.close()
    GPIO.cleanup()
##try:
##    while True:
##        GPIO.output(pin, True)
##        packet_type = [int(datum) for datum in input("> ").split(",")]
##        print("packet_type:", packet_type)
##        spi.xfer2(packet_type)
##        
##        if packet_type == [2]:
##            data_len = spi.xfer2([0])
##            print("data_len:", data_len)
##            data_len = data_len[0]
##            response = spi.xfer2([0] * data_len)
##            print(response)
##        else:
##            data = [int(datum) for datum in input("> ").split(",")]
##            print("response:", [chr(x) for x in spi.xfer2(data)])
##        GPIO.output(pin, False)
##
##except KeyboardInterrupt:
##    spi.close()
