
import pyb

data = b''

def receive(line):
    global data
    print("receiving...")
    try:
        data_len = int(spi.send_recv(b'0', timeout=50000))
        data = spi.send_recv(b'0' * data_len, timeout=50000)
        print(data)
    except OSError:
        print("No data received")


spi = pyb.SPI(1, pyb.SPI.SLAVE, baudrate=600000, polarity=1, phase=0, crc=0x7)

extint = pyb.ExtInt("X5", pyb.ExtInt.IRQ_FALLING, pyb.Pin.PULL_UP, receive)

while True:
    spi.send_recv(5, timeout=50000)
    spi.send_recv(pyb.rng(), timeout=50000)  # generate 5 random characters
    pyb.delay(500)
