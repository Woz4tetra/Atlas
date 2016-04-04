from Adafruit.Adafruit_I2C.Adafruit_I2C.Adafruit_I2C as AdaI2C

class I2C:
    def __init__(self, address, **registers):
        self.address = address
        self.registers = registers
        
        self.i2c = AdaI2C(self.address)
    
