import traceback
import sys
import time

sys.path.insert(0, '../')

from board.data import Command
from board.data import start, stop

servo = Command(0, 'position', (-90, 90), bound=False)

start(use_handshake=False)

try:
    while is_running():
        position = int(input('servo:'))
        servo["position"] = position
        print(servo["position"])
        # for counter in range(180):
        #     servo["position"] = position + counter
        #     time.sleep(0.005)
        # print(servo["position"])
except:
    traceback.print_exc()
    stop()
