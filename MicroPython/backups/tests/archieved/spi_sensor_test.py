
import pyb

new_data = False
def receive(line):
    global new_data
    print("!!")
    new_data = True

spi = pyb.SPI(1, pyb.SPI.SLAVE, baudrate=600000, polarity=1, phase=0, crc=0x7)

extint = pyb.ExtInt("X4", pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_UP, receive)

try:
    while True:
        # spi.send_recv(5, timeout=50000)
        # spi.send_recv(pyb.rng(), timeout=50000)  # generate 5 random characters
        # print(".", end="")
        print(".")
        if new_data:
            print("receiving...")
            # # try:
            # packet = [_ for _ in spi.recv(bytearray(b'000000'), timeout=50000)]
            packet = [_ for _ in spi.recv(6)]
            print("packet:", [chr(_) for _ in packet])

            if packet[0] == 1:
                # expect to receive command
                # if spi broke, it's unlikely it will return 1
                print("command")
                print("command ID:", packet[1])
                print("data:", bytes(packet[2:]))
            elif packet[0] == 2:
                # send sensor data
                print("sensor")
                print("sensor ID:", packet[1])

                data = bytes(packet[0:2])
                for _ in range(4):
                    data += chr(pyb.rng() & 0xff)

                print("data:", [_ for _ in data])
                spi.send(bytearray(data), timeout=50000)
                # print("received:", data)
            else:
                print("invalid packet type:", packet)
            new_data = False
        pyb.delay(500)
        pyb.LED(1).toggle()
except KeyboardInterrupt:
    spi.deinit()
