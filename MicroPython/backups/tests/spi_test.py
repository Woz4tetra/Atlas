
import pyb

spi = pyb.SPI(1, pyb.SPI.SLAVE, baudrate=600000, polarity=1, phase=0, crc=0x7)

while True:
    print("waiting for data length")
    data_recved = False
    data_len = b'0'
    while not data_recved:
        try:
            data_len = int(spi.send_recv(data_len, timeout=50000))
            data_recved = True
        except OSError:
            pyb.delay(1000)

    data_recved = False
    data = b'0' * data_len
    print("data length:", data_len)
    pyb.LED(1).toggle()

    try:
        data = spi.send_recv(data, timeout=50000)
        pyb.LED(2).toggle()
    except OSError:
        pyb.delay(1000)
    print("data:", data)
    pyb.delay(500)
