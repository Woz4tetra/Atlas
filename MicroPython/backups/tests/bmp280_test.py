
import pyb
from libraries.bmp280 import *

bmp = BMP280_I2C(2)

while True:
    print(bmp.altitude(), bmp.temperature(), bmp.pressure())
#    bmp.temperature()
    pyb.delay(50)
