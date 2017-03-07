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
        
        self.paused = True
        
        self.who_i_am = "dummy"  # put who_i_am ID here
        self.packet_end = "\n"
        
        self.accel = pyb.Accel()
        self.switch = pyb.Switch()
        self.switch.callback(lambda: self.write_switch())
        self.should_write_switch = False
        
        self.leds = [pyb.LED(x + 1) for x in range(4)]

        self.led_colors = {
            'r': 0, 'g': 1, 'y': 2, 'b': 3
        }

        self.leds[0].on()
        
        self.py_version = sys.version
        self.upy_version = "%d.%d.%d" % sys.implementation[1]
        
        self.time0 = time.ticks_us()
        self.write_time0 = time.ticks_ms()

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
                    self.time0 = time.ticks_us()
                
                elif packet == "start!":
                    self.paused = False
                    
                elif packet == "stop":
                    self.paused = True
                    
                    self.all_off()
                    self.leds[0].on()
                    
                elif len(packet) > 0:
                    if packet[1:].isdigit():
                        value = int(packet[1:])
                        self.leds[self.led_colors[packet[0]]].intensity(value)
    
    def write_switch(self):
        self.should_write_switch = True
    
    def write_sensors(self):
        if self.should_write_switch:
            self.write("s")
            self.should_write_switch = False
            
        if time.ticks_diff(self.write_time0, time.ticks_ms()) > 15:
            accel_data = "%i\t%i\t%i" % self.accel.filtered_xyz()
            dt = time.ticks_diff(self.time0, time.ticks_us()) * 1E-6
            timestamp = "%s\t" % dt
            self.write("a" + timestamp + accel_data)
            
            self.write_time0 = time.ticks_ms()
    
    def run(self):
        while True:
            if self.paused:
                pyb.delay(100)
            else:
                self.write_sensors()
            
            self.read_commands()
            pyb.delay(1)
            
            
    # -----

    def all_off(self):
        for led in self.leds:
            led.off()

RobotInterface().run()
