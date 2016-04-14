import pyb
import struct
import math


class BNO055():
    reg = dict(
        VECTOR_ACCELEROMETER = 0x08,
        VECTOR_MAGNETOMETER = 0x0e,
        VECTOR_GYROSCOPE = 0x14,
        VECTOR_EULER = 0x1a,
        VECTOR_LINEARACCEL = 0x28,
        VECTOR_GRAVITY = 0x2e,
        QUATERNION_DATA = 0x20,
        TEMPERATURE = 0x34,

        CHIP_ID = 0x00,
        SYS_TRIGGER = 0x3f,
        OPR_MODE = 0x3d,
        PAGE_ID = 0x07,
        PWR_MODE = 0x3e,
    )

    modes = dict(
        CONFIG = 0x00,
        NDOF = 0x0c,
    )

    power_modes = dict(
        NORMAL = 0x00,
        LOW = 0x01,
        SUSPEND = 0x02
    )

    BNO055_ID = 0xa0

    def __init__(self, bus, default_address=True, declination=(0, 0)):
        self.i2c = pyb.I2C(bus, pyb.I2C.MASTER)
        if default_address:
            self.address = 0x28
        else:
            self.address = 0x29

        self.declination = (declination[0] + declination[1] / 60) * math.pi / 180

        self.quat_scale = 1.0 / (1 << 14)
        self.sample_delay = 100

        addresses = self.i2c.scan()

        if self.address not in addresses:
            raise Exception("Address not found during scan: " + str(addresses))

        if not self.i2c.is_ready(self.address):
            raise Exception("Device not ready")
        pyb.delay(50)
        chip_id = self.read_8(self.reg['CHIP_ID'])
        if ord(chip_id) != self.BNO055_ID:
            pyb.delay(1000)  # wait for boot
            chip_id = self.read_8(self.reg['CHIP_ID'])
            if ord(chip_id) != self.BNO055_ID:
                raise Exception("Chip ID invalid:", chip_id)

        self.set_mode(self.modes['CONFIG'])

        self.write_8(self.reg['SYS_TRIGGER'], 0x20)  # reset
        pyb.delay(1000)
        # while ord(self.read_8(self.reg['CHIP_ID'])) != self.BNO055_ID:
        #     pyb.delay(10)
        self.write_8(self.reg['PWR_MODE'], self.power_modes['NORMAL'])
        pyb.delay(10)

        self.write_8(self.reg['PAGE_ID'], 0)

        self.write_8(self.reg['SYS_TRIGGER'], 0x0)
        pyb.delay(10)

        self.set_mode(self.modes['NDOF'])
        pyb.delay(20)

        pyb.delay(1000)

        self.set_ext_crystal_use()

    def set_mode(self, mode):
        self.write_8(self.reg['OPR_MODE'], mode)
        pyb.delay(30)

    def set_ext_crystal_use(self):
        self.set_mode(self.modes['CONFIG'])
        pyb.delay(25)

        self.write_8(self.reg['PAGE_ID'], 0)
        self.write_8(self.reg['SYS_TRIGGER'], 0x80)
        pyb.delay(10)

        self.set_mode(self.modes['NDOF'])
        pyb.delay(20)

    def get_lin_accel(self):
        x, y, z = self.get_vector('VECTOR_LINEARACCEL')
        return x / 100.0, y / 100.0, z / 100.0

    def get_gyro(self):
        x, y, z = self.get_vector('VECTOR_GYROSCOPE')
        return x / 900.0, y / 900.0, z / 900.0

    def get_quat(self):
        buf = self.read_len(self.reg['QUATERNION_DATA'], 8)
        w = (buf[1] << 8) | buf[0]
        x = (buf[3] << 8) | buf[2]
        y = (buf[5] << 8) | buf[4]
        z = (buf[7] << 8) | buf[6]
        return (w * self.quat_scale,
                x * self.quat_scale,
                y * self.quat_scale,
                z * self.quat_scale)

    def get_euler(self):
        x, y, z = self.get_vector('VECTOR_EULER')
        # return x, y, z
        return x / 16.0, y / 16.0, z / 16.0


    def get_temp(self):
        return ord(self.read_8(self.reg['TEMPERATURE']))

    def get_accel(self):
        x, y, z = self.get_vector('VECTOR_ACCELEROMETER')
        return x / 100.0, y / 100.0, z / 100.0

    def get_grav(self):
        x, y, z = self.get_vector('VECTOR_GRAVITY')
        return x / 100.0, y / 100.0, z / 100.0

    def get_mag(self):
        x, y, z = self.get_vector('VECTOR_MAGNETOMETER')
        return x / 16.0, y / 16.0, z / 16.0

    def get_vector(self, vector_type):
        buf = self.read_len(self.reg[vector_type], 6)
        data = []
        for index in range(0, len(buf), 2):
            datum = (buf[index + 1] << 8) | buf[index]
            if datum > 0x7fff:
                datum -= 0x10000
            data.append(datum)
        return data

    def get_heading(self):
        x, y, z = self.get_mag()
        heading = math.atan2(y, x)
        heading += self.declination
        # Correct for reversed heading
        if heading < 0:
            heading += 2 * math.pi
        # Check for wrap and compensate
        elif heading > 2 * math.pi:
            heading -= 2 * math.pi
        return heading

    def write_8(self, register, data):
        return self.i2c.mem_write(data, self.address, register)

    def read_8(self, register):
        return self.i2c.mem_read(1, self.address, register)

    def read_len(self, register, length):
        return self.i2c.mem_read(length, self.address, register)
