# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb

pyb.usb_mode('CDC+MSC')

# pyb.main('pi_main.py')  # main script to run after this on
# pyb.main("tests/lidar_turret_test.py")
pyb.main("tests/bno055_test.py")
# pyb.main('tests/bmp280_test.py')
