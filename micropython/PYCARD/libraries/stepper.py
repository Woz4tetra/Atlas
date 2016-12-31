import time
import pyb

class Stepper:
    def __init__(self, step_num, speed, phase_a1_pin, phase_a2_pin, phase_b1_pin, phase_b2_pin):
        self.motor_pin_1 = pyb.Pin(phase_a1_pin, pyb.Pin.OUT_PP)
        self.motor_pin_2 = pyb.Pin(phase_a2_pin, pyb.Pin.OUT_PP)
        self.motor_pin_3 = pyb.Pin(phase_b1_pin, pyb.Pin.OUT_PP)
        self.motor_pin_4 = pyb.Pin(phase_b2_pin, pyb.Pin.OUT_PP)
        
        self.step_number = 0
        self.direction = False
        self.last_step_time = 0
        self.step_num = step_num
        self.step_delay = 0
        
        self.set_speed(speed)
        
        self.time0 = time.ticks_ms()
        self.delay = self.step_delay / 2000
        
    def set_speed(self, speed):  # in rpm
        self.step_delay = int(6E7 / self.step_num / speed)
    
    
    def step(self, num_steps):
        if time.ticks_diff(self.time0, time.ticks_ms()) > self.delay:
            steps_left = abs(num_steps)
            
            if num_steps > 0: self.direction = True
            if num_steps < 0: self.direction = False
            
            # decrement the number of steps, moving one step each time:
            while steps_left > 0:
                now = time.ticks_us()
                
                # move only if the appropriate delay has passed:
                if time.ticks_diff(self.last_step_time, now) >= self.step_delay:
                    self.last_step_time = now
                    
                    if self.direction:
                        self.step_number += 1
                        if self.step_number == self.step_num:
                            self.step_number = 0
                    else:
                        if self.step_number == 0:
                           self.step_number = self.step_num
                        self.step_number -= 1
                    steps_left -= 1
                    
                    self.step_motor(self.step_number % 4)
            self.time0 = time.ticks_ms()
    
    def step_motor(self, this_step):
        if this_step == 0:  # 1010
            self.motor_pin_1.high()
            self.motor_pin_2.low()
            self.motor_pin_3.high()
            self.motor_pin_4.low()
        elif this_step == 1:  # 0110
            self.motor_pin_1.low()
            self.motor_pin_2.high()
            self.motor_pin_3.high()
            self.motor_pin_4.low()
        elif this_step == 2:  # 0101
            self.motor_pin_1.low()
            self.motor_pin_2.high()
            self.motor_pin_3.low()
            self.motor_pin_4.high()
        elif this_step == 3:  # 1001
            self.motor_pin_1.high()
            self.motor_pin_2.low()
            self.motor_pin_3.low()
            self.motor_pin_4.high()
