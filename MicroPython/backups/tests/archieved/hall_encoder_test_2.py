import pyb
from objects import HallEncoder

pin = "X11"

mode = ""
while mode != "e" and mode != "r":
    mode = input("Raw or encoder counts (r or e)?\n> ").lower()

if mode == "e":
    encoder = HallEncoder(0, pin, 80, 100)
    while True:
        if encoder.recved_data():
            print(encoder.enc_dist, encoder.hall_value)
        pyb.delay(100)

elif mode == "r":
    pin_ref = pyb.ADC(pyb.Pin(pin, pyb.Pin.ANALOG))
    while True:
        print(pin_ref.read())
        pyb.delay(40)
