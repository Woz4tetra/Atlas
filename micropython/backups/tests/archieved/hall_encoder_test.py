import pyb

class HallEncoder(object):
    def __init__(self, analog_pin):
        self.pin_ref = (
            pyb.ADC(pyb.Pin(analog_pin, pyb.Pin.ANALOG)))

        self.in_range = False
        self.enc_dist = 0
        self.timer1 = pyb.Timer(4, freq=50)
        self.timer1.callback(lambda t: self.on_interrupt())

        # need to be calibrated to real life values
        self.upper_threshold = 3900
        self.lower_threshold = 3600

        self.data_recved = False

	def recved_data(self):
	    if self.data_recved == True:
	        self.data_recved = False
	        return True
	    else:
	        return False

    def on_interrupt(self):
        self.hall_value = self.pin_ref.read()

        if (self.in_range and (self.hall_value > self.upper_threshold)):
            self.in_range = False
            self.enc_dist += 1
            self.data_recved = True
        elif (not self.in_range and (self.hall_value <= self.lower_threshold)):
            self.in_range = True

encoder = HallEncoder("X7")
while True:
	# if encoder.data_recved:
		print(encoder.enc_dist)
