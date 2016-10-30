
from objects import GPS

gps = GPS(1, 6, "Y3")

while True:
    pyb.delay(1)
    if gps.recved_data():
        gps.stream_data()
        gps.update_data()
        print(gps.lat,
              gps.long,
              gps.gps_ref.satellites_in_view)