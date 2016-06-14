import smbus
import time

bus = smbus.SMBus(1)
address = 0x12

def end():
    bus.close()

def read(length):
    for _ in range(length):
        print(bus.read_byte(address), end="")
    print()

def write(*data):
    bus.write_i2c_block_data(address, 0, list(data))
