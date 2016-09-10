import pyb

servo = pyb.Servo(1)
ac = pyb.Accel()
k = 3

while True:
  pyb.delay(250)
  servo.angle(ac.x()*k)
