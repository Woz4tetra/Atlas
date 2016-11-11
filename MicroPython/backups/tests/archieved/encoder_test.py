class HallEncoder(object):
    def __init__(self, analog_pin):

	self.pin_ref = (
		pyb.ADC(pyb.Pin(analog_pin,pyb.Pin.ANALOG)))

	self.in_range = False
	self.enc_dist = 0
	self.timer1 = pyb.Timer(4)
	self.timer1.init(freq=50)
	self.timer1.callback(lambda t: HallEncoder.on_interrupt(self))
	self.time = 0

	#need to be calibrated to real life values
	self.upper_threshold = 950
	self.lower_threshold = 850
    @staticmethod
    def on_interrupt(self):
	print("hi there")
	hall_value = self.pin_ref.read()
	self.time +=1

	if (self.in_range and (hall_value > self.upper_threshold)):
	    self.in_range = False
	    self.enc_dist += 1
	elif (not(in_range) and (hall_value <= self.lower_threshold)):
	    self.in_range = True


    def update_data(self):
	self.data = [self.enc_dist]


obj = HallEncoder("X8")
while True:
	print(obj.enc_dist, obj.time)
	print("1")
