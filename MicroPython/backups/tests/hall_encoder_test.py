import pyb



#set up variables
num_rotations = 0
in_range = False
hall_value = 0
count = 0

#constants
upper_threshold = 3900
lower_threshold = 3100
pin_ref = pyb.ADC(pyb.Pin('X7', pyb.Pin.ANALOG))

#make interrupt function

def update():
	global pin_ref
	global in_range
	global upper_threshold
	global num_rotations
	global lower_threshold
	global hall_value
	global count

	hall_value = pin_ref.read()

	#this allows for the hall effect vlue to be visually tested 
	#every 0.5 seconds for finding thresholds
	count += 1
	if (count % 100 == 0):
		print (hall_value, count)

	#this prevents multiple readings of the same rotation
	if (in_range and (hall_value > upper_threshold)):
	    in_range = False
	    num_rotations += 1
	elif (not(in_range) and (hall_value <= lower_threshold)):
	    in_range = True


#set up timer for interrupt
timer1 = pyb.Timer(4, freq = 200)
timer1.callback(lambda t: update())

#loop
print("starting the loop")
while True:
	print(num_rotations, 'number of rotation')
	pyb.delay(1000)



