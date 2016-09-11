from pyb import I2C

i2c = I2C(2, I2C.SLAVE)
i2c.init(I2C.SLAVE)

while True:
    print("waiting...")
    print(i2c.recv(3, timeout=-1))  ## no timeout
    pyb.delay(250)
