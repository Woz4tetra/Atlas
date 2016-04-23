from .i2c import I2C

class MCP9808(I2C):
    def __init__(self):
        super(MCP9808, self).__init__(
            0x18,
            {
                "config": 0x01,
                "config shutdown": 0x0100,
                "config critlocked": 0x0080,
                "config winlocked": 0x0040,
                "config intclr": 0x0020,
                "config alertstat": 0x0010,
                "config alertctrl": 0x0008,
                "config alertsel": 0x0004,
                "config alertpol": 0x0002,
                "config alertmode": 0x0001,
                "upper temp": 0x02,
                "lower temp": 0x03,
                "crit temp": 0x04,
                "ambient temp": 0x05,
                "manuf id": 0x06,
                "device id ": 0x07
            }
        )

    def begin(self):
        if self.readU16("manuf id") != 0x0054:
            return "Manufacturer ID did not match: " + str(
                self.readU16("manuf id"))

        if self.readU16("device id") != 0x0400:
            return "Device ID did not match: " + str(
                self.readU16("device id"))

        return 0

    def temp_f(self):
        return self.temp_c() * 9 / 5 + 32

    def temp_c(self):
        raw = self.readU16("ambient temp")
        temperature = raw & 0x0fff
        temperature /= 16.0
        if raw & 0x1000: temperature -= 0x100;

        return temperature

    def shutdown_wake(self, shutdown=True):
        conf_register = self.readU16("config")

        if shutdown:
            conf_shutdown = conf_register | self.registers["config shutdown"]
        else:
            conf_shutdown = conf_register ^ self.registers["config shutdown"]
        self.write16("config", conf_shutdown)

