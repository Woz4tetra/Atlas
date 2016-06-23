import time
import sys

sys.path.insert(0, '../')

from microcontroller.comm import *

def read_for(seconds):
    global communicator
    start_time = time.time()
    while time.time() - start_time < seconds:
        time.sleep(0.0005)
        print(communicator.serial_ref.read().decode('ascii'), end="")

communicator = Communicator(None, log_data=False, handshake=False)

data = ""

print("Type start for control-D")
print("Type stop for control-C")
print("Type stop start to restart the pyboard")
print("Type exit or quit to end the program")
while data != 'exit' and data != 'quit':
    data = input(">> ")
    for command in data.split(" "):
        if data == "start":
            communicator.write_byte(bytes(chr(4), encoding='ascii'))
            read_for(3)
        elif data == "stop":
            communicator.write_byte(bytes(chr(3), encoding='ascii'))
            time.sleep(0.25)
            communicator.write_byte(bytes(chr(3), encoding='ascii'))
            time.sleep(0.25)
            read_for(0.5)
        elif data == "quit" or data == "exit":
            break
        else:
            read_for(1)
        
communicator.serial_ref.close()
