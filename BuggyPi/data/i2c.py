from data.Adafruit.Adafruit_I2C.Adafruit_I2C import Adafruit_I2C as AdaI2C


class I2C:
    def __init__(self, addresses, registers, constants=None, default_addr=0):
        self.addresses = addresses
        try:
            self.address = addresses[default_addr]
        except TypeError:  # if not a list
            self.address = addresses
        self.registers = registers
        self.constants = constants

        self.i2c = AdaI2C(self.address)

        output = self.begin()
        if type(output) == str:
            raise Exception(output)

    def begin(self):
        return 0

    def write8(self, reg, value):
        if value in self.constants:
            value = self.constants[value]
        self.write8(self.registers[reg], value)

    def write16(self, reg, value):
        if value in self.constants:
            value = self.constants[value]
        self.write16(self.registers[reg], value)

    def writeRaw8(self, value):
        if value in self.constants:
            value = self.constants[value]
        self.writeRaw8(value)

    def writeList(self, reg, value_list):
        for index in range(len(value_list)):
            if value_list[index] in self.constants:
                value_list[index] = self.constants[value_list[index]]
        self.writeList(self.registers[reg], value_list)

    def readList(self, reg, length):
        self.readList(self.registers[reg], length)

    def readU8(self, reg):
        self.readU8(self.registers[reg])

    def readS8(self, reg):
        self.readS8(self.registers[reg])

    def readU16(self, reg, little_endian=True):
        self.readU16(self.registers[reg], little_endian)

    def readS16(self, reg, little_endian=True):
        self.readS16(self.registers[reg], little_endian)
