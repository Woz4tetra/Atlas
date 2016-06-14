import pyb
from libraries.rc_motors import RCmotors

motors = RCmotors("X8", 40, 850, 40)
motors.set_speed(-40)

try:
    while True:
        if motors.new_encoder_data():
            print(motors.enc_dist)
        
        pyb.delay(50)
except KeyboardInterrupt:
    motors.stop()