import pyb
import time
import sys
import struct
import array
import time

class RobotInterface:
    def __init__(self):
        self.serial_ref = pyb.USB_VCP()
        self.buffer = ""
        
        self.enable_write = False
        self.enable_read = False
        
        self.who_i_am = "dummy"  # put who_i_am ID here
        self.packet_end = "\n"
        
        self.accel = pyb.Accel()
        self.leds = [pyb.LED(x + 1) for x in range(4)]

        self.led_colors = {
            'r': 0, 'g': 1, 'y': 2, 'b': 3
        }

        self.leds[0].on()
        
        self.py_version = sys.version
        self.upy_version = "%d.%d.%d" % sys.implementation[1]
        
        self.time0 = time.ticks_us()

    def read_packets(self):
        incoming = self.serial_ref.read(self.serial_ref.any())
        self.buffer += incoming.decode('ascii')
        packets = self.buffer.split(self.packet_end)

        if len(self.buffer) > len(self.packet_end):
            self.buffer = packets.pop(-1)
        
        return packets

    @staticmethod
    def float_to_hex(number):
        return "%0.8x" % (struct.unpack('<I', bytes(array.array('f', [number]))))
        
    def write(self, packet):
        packet += self.packet_end
        self.serial_ref.write(packet.encode('ascii'))

    def ready(self):
#        self.write("ready!")
        
        self.all_off()
        self.leds[1].on()
        
    @staticmethod
    def hex_to_float(hex_string):
        return struct.unpack('!f', bytes.fromhex(hex_string))[0]
    
    def read_commands(self):
        if self.serial_ref.any():
            for packet in self.read_packets():
#                if packet == "ready?":
#                    self.ready()
                    
                if packet == "whoareyou":
#                    self.write("iam%s\t%s\t%s\t%i\t%i\t%i\t%i" % (self.who_i_am,
#                        self.py_version, self.upy_version, 0, 1, 0, 0))
                    self.write("iam%s" % (self.who_i_am))
                    self.ready()

                elif packet == "init?":
                    self.write("init:%s\t%s\t%i\t%i\t%i\t%i" % (
                        self.py_version, self.upy_version, 0, 1, 0, 0))
                    self.enable_read = True
                    self.enable_write = True
                    self.time0 = time.ticks_us()
                    
                elif packet == "stop":
                    self.enable_write = False
                    self.enable_read = False
                    
                    self.all_off()
                    self.leds[0].on()
                elif len(packet) > 0:
                    if packet[1:].isdigit():
                        value = int(packet[1:])
                        self.leds[self.led_colors[packet[0]]].intensity(value)
    
    def write_sensors(self):
        accel_data = "%i\t%i\t%i" % self.accel.filtered_xyz()
        dt = time.ticks_diff(self.time0, time.ticks_us()) * 1E-6
        timestamp = "%s\t" % dt
        self.write(timestamp + accel_data)
    
    def run(self):
        while True:
            if self.enable_write:
                self.write_sensors()
            
            if not self.enable_read:
                pyb.delay(100)
            
            self.read_commands()
            
            
    # -----

    def all_off(self):
        for led in self.leds:
            led.off()

RobotInterface().run()
