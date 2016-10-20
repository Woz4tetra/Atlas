# boot.py -- run on boot-up
# can run arbitrary Python, but best to keep it minimal

import pyb

pyb.usb_mode('CDC+MSC')

file_name = 'quasar_main.py'
print(file_name)

pyb.main(file_name)  # main script to run after this on
