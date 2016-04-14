# main.py -- put your code here!

import pyb
from pyb import UART
from objects import *
from data import *
from comm import Communicator
from logger import Recorder

# tmp36 = TMP36(0, pyb.Pin.board.Y12)
# mcp9808 = MCP9808(1, 1)
# accel = BuiltInAccel(2)
gps = GPS(1)
# accel_gyro = AccelGyro(4, 1)
# compass = Compass(5, 1)
encoder = HallEncoder(2, "X7")
imu = IMU(3, 2)

# servo_steering = I2CServo(2, 0, 0)
# servo_brakes = I2CServo(2, 1, 1)

servo_steering = Servo(0, 1, 0)
# motor_a = Motor(1, 'X2', 'X3')

gps_indicator = pyb.LED(3)

new_data = False


def pps_callback(line):
    global new_data, gps_indicator
    new_data = True
    gps_indicator.toggle()


uart = UART(6, 9600, read_buf_len=1000)
pps_pin = pyb.Pin.board.X8
extint = pyb.ExtInt(pps_pin, pyb.ExtInt.IRQ_FALLING,
                    pyb.Pin.PULL_UP, pps_callback)

indicator = pyb.LED(4)
increase = True

sensor_queue = SensorQueue(
        gps,
        encoder,
        imu
)
command_pool = CommandPool(
        servo_steering,
        # servo_brakes
)

log_data = False
log = None

def toggle_log():
    global log, log_data

    time0 = pyb.millis()
    while (pyb.millis() - time0) < 500:
        pyb.LED(1).toggle()
        pyb.delay(50)

    if not log_data:
        log = Recorder(frequency=0.01)
        log.add_tracker(gps, "gps")
        log.add_tracker(encoder, "encoder")
        log.add_tracker(imu, "imu")
        log.add_tracker(servo_steering, "servo")
        log.end_init()

        pyb.LED(1).on()
        log_data = True
        log.reset_time()
    else:
        log.close()
        log_data = False
        pyb.LED(1).off()

communicator = Communicator(sensor_queue, command_pool)

while True:
    if new_data:
        while uart.any():
            gps.update(chr(uart.readchar()))
        communicator.write_packet(gps)

    communicator.write_packet(imu)
    if encoder.recved_data():
        communicator.write_packet(encoder)

    if pyb.Switch()():
        toggle_log()

    new_data = False

    communicator.read_command()

    if increase:
        indicator.intensity(indicator.intensity() + 5)
    else:
        indicator.intensity(indicator.intensity() - 5)

    if indicator.intensity() <= 0:
        increase = True
    elif indicator.intensity() >= 255:
        increase = False

    if log_data:
        log.add_data(gps)
        log.add_data(encoder)
        log.add_data(imu)
        log.add_data(servo_steering)
        log.end_row()
    else:
        pyb.delay(5)
