# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb

pyb.usb_mode('CDC+MSC')

pyb.main('quasar/quasar_main.py')  # main script to run after this on
# pyb.main('turret/turret_main.py')
# pyb.main("tests/lidar_turret_test.py")
#pyb.main("tests/bno055_test.py")
# pyb.main("tests/gps_test.py")
# pyb.main("tests/stepper_test.py")
