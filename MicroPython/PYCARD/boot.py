# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb

# pyb.LED(3).on()                 # indicate we are waiting for switch press
# pyb.delay(2000)                 # wait for user to maybe press the switch
# switch_value = pyb.Switch()()   # sample the switch at end of delay
# pyb.LED(3).off()                # indicate that we finished waiting for the switch
#
# pyb.LED(4).on()                 # indicate that we are selecting the mode
#
# if switch_value:
#     pyb.usb_mode('CDC+HID')
# else:
#     pyb.usb_mode('CDC+MSC')
#
# pyb.LED(4).off()
# time0 = pyb.millis()
# while (pyb.millis() - time0) < 500:
#     pyb.LED(4).toggle()
#     pyb.delay(50)

pyb.usb_mode('CDC+MSC')

pyb.main('pi_main.py') # main script to run after this on
##pyb.main("tests/encoder_motor_test.py")
#pyb.main("tests/gps_test.py")
##pyb.main('tests/bmp280_test.py')
