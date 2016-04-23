import sys
import time

sys.path.insert(0, "../")

from data.Adafruit.Adafruit_I2C.Adafruit_I2C import Adafruit_I2C

ambient_temp = 0x05
mcp9808 = Adafruit_I2C(0x18)

while True:
    raw = mcp9808.reverseByteOrder(mcp9808.readU16(ambient_temp))
    temperature = raw & 0x0fff
    temperature /= 16.0
    if raw & 0x1000:
        temperature -= 0x100
    
    print(temperature, raw)
    time.sleep(0.5)
