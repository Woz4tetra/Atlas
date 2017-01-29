import serial
import serial.tools.list_ports
import pprint
import time
from threading import Event, Thread

class Arduino(Thread):
    def __init__(self, address, baudrate):
        self.serial_ref = serial.Serial(address, baudrate=baudrate)
        self.exit_event = Event()
        super(Arduino, self).__init__()
    
    def run(self):
        while not self.exit_event.is_set():
            if self.serial_ref.in_waiting > 0:
                print(self.serial_ref.read(self.serial_ref.in_waiting).decode('ascii'))

            time.sleep(0.1)
    
    def write_loop(self):
        try:
            while True:
                send = input("> ")
                data = bytearray(send, 'ascii')
                self.serial_ref.write(data)
        except KeyboardInterrupt:
            self.exit_event.set()
            self.serial_ref.close()

for port in serial.tools.list_ports.comports():
    pprint.pprint(port.__dict__)

arduino = Arduino("/dev/cu.usbserial-00002014", 115200)
arduino.start()
arduino.write_loop()
