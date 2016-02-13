# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb
pyb.main('tests/servo_test.py') # main script to run after this one
#pyb.main('test_pwm.py')

pyb.usb_mode('CDC+MSC') # act as a serial and a storage device
#pyb.usb_mode('CDC+HID') # act as a serial device and a mouse
