import pyb


class RCmotors():
    def __init__(self, encoder_pin, lower, upper, min_speed):
        self.enc_pin_ref = pyb.ADC(pyb.Pin(encoder_pin, pyb.Pin.ANALOG))
        
        self.in_range = False
        self.enc_dist = 0
        self.hall_value = 0
        
        self.sum = 0
        self.count = 0
        
        # need to be calibrated to real life values
        # for RoboQuasar, upper = 3100, lower = 2900
        self.upper = upper
        self.lower = lower
        
        assert self.lower < self.upper
        
        self.data_recved = False
        
        timer12 = pyb.Timer(12, freq=5000)
        timer12.callback(lambda t: self.on_interrupt())
        
        assert encoder_pin == "X8" or encoder_pin == "Y11" or encoder_pin == "Y12"

        # ----- motor pin inits -----
        timer8 = pyb.Timer(8, freq=1000)
        timer2 = pyb.Timer(2, freq=1000)

        width = 16000
        
        self.pin1 = timer8.channel(3, pyb.Timer.PWM, pin=pyb.Pin.board.Y12, pulse_width=width)
        self.pin2 = timer8.channel(2, pyb.Timer.PWM, pin=pyb.Pin.board.Y11, pulse_width=width)
        self.pin3 = timer2.channel(4, pyb.Timer.PWM, pin=pyb.Pin.board.Y10, pulse_width=width)
        self.pin4 = timer2.channel(3, pyb.Timer.PWM, pin=pyb.Pin.board.Y9, pulse_width=width)
    
        self.speed = 0
        self.min_speed = min_speed
    
    def on_interrupt(self):
        self.hall_value = self.enc_pin_ref.read()
        self.sum += self.hall_value
        self.count += 1
        
        if self.count == 20:
            average = self.sum // self.count
            
            if self.in_range and (average > self.upper):
                self.in_range = False
                
                if self.speed > 0:
                    self.enc_dist += 1
                elif self.speed < 0:
                    self.enc_dist -= 1
                # else: don't record encoder data
                
                self.data_recved = True
            elif not self.in_range and (average <= self.lower):
                self.in_range = True
            
            self.sum = 0
            self.count = 0
    
    def new_encoder_data(self):
        if self.data_recved == True:
            self.data_recved = False
            return True
        else:
            return False
    
    def forward(self, speed):
        speed = abs(speed)
        
        self.pin1.pulse_width_percent(0)
        self.pin3.pulse_width_percent(0)

        self.pin2.pulse_width_percent(speed)
        self.pin4.pulse_width_percent(speed)
    
    def backward(self, speed):
        speed = abs(speed)
        
        self.pin2.pulse_width_percent(0)
        self.pin4.pulse_width_percent(0)
        
        self.pin1.pulse_width_percent(speed)
        self.pin3.pulse_width_percent(speed)
    
    def stop(self):
        self.speed = 0
        
        self.pin2.pulse_width_percent(0)
        self.pin4.pulse_width_percent(0)
        
        self.pin1.pulse_width_percent(0)
        self.pin3.pulse_width_percent(0)
    
    def set_speed(self, speed):
        if abs(speed) > 100:
            speed = 100 * ((speed > 0) - (speed < 0))  # fast way to extract sign
        elif abs(speed) < self.min_speed:
            speed = self.min_speed * ((speed > 0) - (speed < 0))
        self.speed = speed
        
        if speed < 0:
            self.backward(speed)
        elif speed > 0:
            self.forward(speed)
        else:
            self.stop()
