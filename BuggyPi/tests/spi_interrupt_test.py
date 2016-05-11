import spidev
import time

data = b''

def receive(channel):
    global data
    print("receiving...")
    data_length = ord(self.spi.xfer2(b'0'))
    data = self.spi.xfer2(b'0' * data_length)
    print(data)

spi = spidev.SpiDev()
spi.open(0, 0)

GPIO.setmode(GPIO.BCM)

GPIO.add_event_detect(8, GPIO.RISING, callback=receive,
                      bouncetime=300)

try:
    while True:
        data = bytes(input("> "), encoding='ascii') #tuple([int(datum) for datum in input("> ").split(",")])
        print("response:", [ord(x) for x in spi.xfer2(data)])

except KeyboardInterrupt:
    spi.close()
