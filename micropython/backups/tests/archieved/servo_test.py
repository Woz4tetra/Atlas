import pyb

servo = pyb.Servo(1)
while True:
	for angle in range(180):
		servo.angle(angle-90)
		pyb.delay(50)
		if angle % 50 == 0:
			print(angle)

