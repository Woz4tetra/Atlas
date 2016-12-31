import pyb
time = 0
enc_dist = 0
in_range = False
upper_threshold = 3600
lower_threshold = 3300
pin_ref = 0

def __init__(analog_pin):
	global time, in_range, enc_dist, pin_ref
	pin_ref = (
		pyb.ADC(pyb.Pin(analog_pin,pyb.Pin.ANALOG)))
	print(time)
	in_range = False
	enc_dist = 0
	timer1 = pyb.Timer(4, freq = 50)
	timer1.callback(lambda t: on_interrupt())

	#need to be calibrated to real life values
	while True:
		print(enc_dist, time, timer1.counter())
		pyb.delay(1000)
	
	
def on_interrupt():
	global time, in_range,upper_threshold
	global lower_threshold, pin_ref, enc_dist
	hall_value = pin_ref.read()
	time +=1
	print(hall_value)

	if (in_range and (hall_value > upper_threshold)):
	    in_range = False
	    enc_dist += 1
	elif (not(in_range) and (hall_value <= lower_threshold)):
	    in_range = True

print("started")
__init__("X8")

