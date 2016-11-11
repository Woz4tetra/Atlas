
from libraries.adafruit_gps import AdafruitGPS

gps = AdafruitGPS(1, 4, 9600, 5)

while True:
    pyb.delay(1)
    if gps.received_sentence():
        print(gps.sentence)
        gps.sentence = ""
