
import serial
import time

serial_ref = serial.Serial(port="/dev/cu.usbserial-00001014", baudrate=9600)

while True:
    prompt = input("Enter 'i' or 'o': ")
    byteprompt = str.encode(prompt)
    serial_ref.write(byteprompt)
    time.sleep(.001)
    data = serial_ref.read()
    decoded = data.decode(encoding='UTF-8')
    print(decoded, end="")
