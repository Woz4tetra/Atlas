import pyb


def to_signed(number, num_bits):
    if (number >> (num_bits - 1)) == 1:
        number -= 2 << (num_bits - 1)
    return number


class BMP280:
    registers = {
        "T1": 0x88, "T2": 0x8A, "T3": 0x8C,
        "P1": 0x8E, "P2": 0x90, "P3": 0x92,
        "P4": 0x94, "P5": 0x96, "P6": 0x98,
        "P7": 0x9A, "P8": 0x9C, "P9": 0x9E,
        "chip id": 0xd0,
        "version": 0xd1,
        "soft reset": 0xe0,
        "cal 26": 0xe1,

        "control": 0xf4,
        "config": 0xf5,
        "pressure": 0xf7,
        "temperature": 0xfa
    }

    def __init__(self):

        self.calib = {
            'T1': 0, 'T2': 0, 'T3': 0,

            'P1': 0, 'P2': 0, 'P3': 0,
            'P4': 0, 'P5': 0, 'P6': 0,
            'P7': 0, 'P8': 0, 'P9': 0,

            'H1': 0, 'H2': 0, 'H3': 0,
            'H4': 0, 'H5': 0, 'H6': 0,
        }

        chip_id = self.read8(self.registers["chip id"])
        if chip_id != 0x58:
            raise Exception("Chip ID invalid: " + str(chip_id))

        self.read_coeffs()

        self.write8(self.registers["control"], 0x3f)

        self.sea_level_hPa = 1013.25

    def read8(self, register):
        return 0

    def write8(self, register, value):
        return 0

    def read16(self, register):
        return 0

    def read16_LE(self, register):
        data = self.read16(register)
        return (data >> 8) | ((data << 8) & 0xff00)

    def readS16(self, register):
        return to_signed(self.read16(register), 16)

    def readS16_LE(self, register):
        return to_signed(self.read16_LE(register), 16)

    def read24(self, register):
        return 0

    def read_coeffs(self):
        self.calib['T1'] = self.read16_LE(self.registers["T1"])
        self.calib['T2'] = self.readS16_LE(self.registers["T2"])
        self.calib['T3'] = self.readS16_LE(self.registers["T3"])

        self.calib['P1'] = self.read16_LE(self.registers["P1"])
        self.calib['P2'] = self.readS16_LE(self.registers["P2"])
        self.calib['P3'] = self.readS16_LE(self.registers["P3"])
        self.calib['P4'] = self.readS16_LE(self.registers["P4"])
        self.calib['P5'] = self.readS16_LE(self.registers["P5"])
        self.calib['P6'] = self.readS16_LE(self.registers["P6"])
        self.calib['P7'] = self.readS16_LE(self.registers["P7"])
        self.calib['P8'] = self.readS16_LE(self.registers["P8"])
        self.calib['P9'] = self.readS16_LE(self.registers["P9"])

    def get_fine_temp(self):
        adc_temp = self.read24(self.registers["temperature"])
        adc_temp >>= 4

        var1 = (adc_temp >> 3) - (self.calib['T1'] << 1)
        var1 *= self.calib['T2']
        var1 >>= 11

        var2 = (adc_temp >> 4) - self.calib['T1']
        var2 *= var2
        var2 >>= 12
        var2 *= self.calib['T3']
        var2 >>= 14

        return var1 + var2

    def temperature(self):
        fine_temp = self.get_fine_temp()

        return ((fine_temp * 5 + 128) >> 8) / 100

    def pressure(self):
        t_fine = self.get_fine_temp()
        adc_P = self.read24(self.registers["pressure"])
        adc_P >>= 4

        var1 = t_fine - 128000
        var2 = var1 * var1 * self.calib['P6']
        var2 += ((var1 * self.calib['P5']) << 17)
        var2 += (self.calib['P4'] << 35)
        var1 = ((var1 * var1 * self.calib['P3'] >> 8) + (
            (var1 * self.calib['P2']) << 12))
        var1 = ((1 << 47) + var1) * (self.calib['P1']) >> 33

        if var1 == 0:
            return 0

        p = 1048576 - adc_P
        p = (((p << 31) - var2) * 3125) // var1
        var1 = ((self.calib['P9']) * (p >> 13) * (p >> 13)) >> 25
        var2 = ((self.calib['P8']) * p) >> 19

        p = ((p + var1 + var2) >> 8) + ((self.calib['P7']) << 4)

        return float(p) / 256

    def altitude(self):
        pressure = self.pressure()
        pressure /= 100
        altitude = 44330 * (1.0 - ((pressure / self.sea_level_hPa) ** 0.1903))

        return altitude


class BMP280_I2C(BMP280):
    def __init__(self, bus, address=0x77):
        self.address = address
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)

        addresses = self.i2c.scan()

        if self.address not in addresses:
            raise Exception("Address not found during scan: " + str(addresses))

        if not self.i2c.is_ready(self.address):
            raise Exception("Device not ready")

        super(BMP280_I2C, self).__init__()

    def read8(self, register):
        #        self.i2c.send(register, self.address)
        #        return ord(self.i2c.recv(1, self.address))
        return ord(self.i2c.mem_read(1, self.address, register))

    def write8(self, register, value):
        self.i2c.mem_write(value, self.address, register)

    def read16(self, register):
        #        self.i2c.send(register, self.address)
        #        data = self.i2c.recv(2, self.address)
        data = self.i2c.mem_read(2, self.address, register)
        return (data[0] << 8) | data[1]

    def read24(self, register):
        #        self.i2c.send(register, self.address)
        #        data = self.i2c.recv(3, self.address)
        data = self.i2c.mem_read(3, self.address, register)
        return (data[0] << 16) | (data[1] << 8) | data[2]


class BMP280_SPI(BMP280):
    def __init__(self, bus):
        #        self.spi = pyb.SPI(bus, mode=pyb.SPI.MASTER, baudrate=500000)

        self.ss = pyb.Pin('Y5', pyb.Pin.OUT_PP)
        self.sck = pyb.Pin('Y6', pyb.Pin.OUT_PP)
        self.miso = pyb.Pin('Y7', pyb.Pin.IN)
        self.mosi = pyb.Pin('Y8', pyb.Pin.OUT_PP)

        super(BMP280_SPI, self).__init__()

    #    def spixfer(self, value):
    #        return ord(self.spi.send_recv(value))

    def spixfer(self, value):
        reply = 0
        for counter in range(7, -1, -1):
            reply <<= 1
            self.sck.low()
            self.mosi.value(value & (1 << counter))
            self.sck.high()
            if self.miso.value():
                reply |= 1
        print(reply)
        return reply

    def read8(self, register):
        self.spixfer(register | 0x80)
        return self.spixfer(0)

    def write8(self, register, value):
        self.spixfer(register & ~0x80)
        self.spixfer(value)

    def read16(self, register):
        self.spixfer(register | 0x80)
        return (self.spixfer(0) << 8) | self.spixfer(0)

    def read24(self, register):
        self.spixfer(register | 0x80)
        data = self.spixfer(0)
        data <<= 8
        data |= self.spixfer(0)
        data <<= 8
        data |= self.spixfer(0)
        data <<= 8

        return data
