import struct
import time

from microcontroller import data
from microcontroller.comm import Communicator


def reset():
    # reset the pyboard if it is in the REPL
    data.communicator.serial_ref.write(struct.pack("B", 4))
    time.sleep(0.01)

    # sets all sensor values to initial conditions (if they have any) and
    # requests data from all sensors
    data.communicator.serial_ref.write(b"R")
    time.sleep(0.01)


def start(baud=115200, use_handshake=True, check_status=False,
          file_name=None, directory=None, log_data=True):

    data.communicator = Communicator(
        baud, data.sensor_pool, use_handshake, log_name=file_name,
        log_dir=directory, log_data=log_data
    )
    data.communicator.start()

    if check_status:
        status = [False] * 5
        status_index = 0

        print("Checking if board is alive...")
        while not all(status):
            if is_running():
                status[status_index] = True
                status_index += 1
            print(".")
            time.sleep(2)
        print("\nIt's alive!")
    reset()


def stop():
    print()
    data.communicator.serial_ref.write(bytes("\x03", encoding='ascii'))
    time.sleep(0.1)
    data.communicator.serial_ref.write(bytes("\x03", encoding='ascii'))
    time.sleep(0.1)
    data.communicator.stop()
    time.sleep(0.005)


def is_running(threshold=1):
    status = (round(time.time() - data.communicator.time0) -
              data.communicator.thread_time) <= threshold
    if status is True:
        data.communicator.time0 = time.time()
    return status


def record(value_name, value):
    if data.communicator.log_data:
        data.communicator.log.enq(value_name, value)
