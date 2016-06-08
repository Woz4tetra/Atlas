
import pyb

new_data = False
def receive(line):
    global new_data
    print("!!")
    new_data = True

spi = pyb.SPI(1, pyb.SPI.SLAVE, baudrate=600000, polarity=1, phase=0, crc=0x7)

extint = pyb.ExtInt("X4", pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_UP, receive)

while True:
    # spi.send_recv(5, timeout=50000)
    # spi.send_recv(pyb.rng(), timeout=50000)  # generate 5 random characters
    # print(".", end="")
    print(new_data)
    if new_data:
        print("receiving...")
        # # try:
        data_len = bytearray(b'0')
        data_len = ord(spi.send_recv(data_len, timeout=50000))
        print("data_len:", data_len)
        data = bytearray(b'0' * data_len)
        data = [_ for _ in spi.send_recv(data, timeout=50000)]
        print(data)
        # except OSError:
        #     print("No data received")
        new_data = False
    pyb.delay(500)
