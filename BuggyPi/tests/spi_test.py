import spidev
import time

spi = spidev.SpiDev()
spi.open(0,0)

try:
    while True:
        data = bytes(input("> "), encoding='ascii') #tuple([int(datum) for datum in input("> ").split(",")])
        print("response:", [chr(int(x)) for x in spi.xfer2(data)])

except KeyboardInterrupt:
    spi.close()
