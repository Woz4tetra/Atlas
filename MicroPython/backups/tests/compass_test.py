import sys

sys.path.insert(0, "../libraries")

from hmc5883l import HMC5883L

compass = HMC5883L(1)
count = 0
while True:
    heading = compass.heading()
    if count % 50 == 0:
        print(heading)
    count += 1
    pyb.delay(5)
