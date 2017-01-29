
import pyb

spi = pyb.SPI(2, pyb.SPI.SLAVE)

while True:
    print("waiting...")
    print([hex(x) for x in spi.recv(2, timeout=50000)])
    pyb.delay(250)
