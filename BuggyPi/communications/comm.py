import RPi.GPIO as GPIO
import spidev


class Communicator:
    def __init__(self):
        self.spi = spidev.SpiDev()
        self.spi.open(0, 0)

        GPIO.setmode(GPIO.BCM)

        GPIO.add_event_detect(25, GPIO.RISING, callback=self.receive,
                              bouncetime=300)

    def receive(self, channel):
        data_length = ord(self.spi.xfer2(b'0'))
        data = self.spi.xfer2(b'0' * data_length)

    def send(self, packet):
        assert type(packet) == bytes
        assert len(packet) <= 255

        self.spi.xfer2(bytes(chr(len(packet)), encoding='ascii'))
        self.spi.xfer2(packet)
