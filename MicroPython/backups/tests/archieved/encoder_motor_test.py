import pyb
from libraries.rc_motors import RCmotors

motors = RCmotors("X8", 50, 900, 40)
motors.set_speed(-80)

try:
    while True:
        if motors.new_encoder_data():
            print(motors.enc_dist)
##        print(motors.hall_value)
        pyb.delay(50)
except KeyboardInterrupt:
    motors.stop()
