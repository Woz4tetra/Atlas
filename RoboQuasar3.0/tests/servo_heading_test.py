import sys
import math
import time
import numpy as np
from matplotlib import pyplot as plt

sys.path.insert(0, "../")

from analyzers.logger import Recorder, get_data
from analyzers.converter import Converter
from analyzers.kalman_filter import HeadingFilter, PositionFilter
from analyzers.binder import Binder
from controllers.servo_map2 import state_to_servo
from controllers.servo_map2 import angle_to_servo

def main(log_data=False):
    if log_data:
        log = Recorder(directory="Test Day 6")

    timestamps, data, length = get_data(
        "Test Day 6/Mon Apr 18 22;03;46 2016.csv",
        ["bind x","bind y","kalman x","kalman y","kalman heading"],
        density=100)
    bind_x, bind_y, k_x, k_y, heading = data


    x, y   = [], []
    gx, gy = [], []
    angle  = []
    lines  = []

    new_x, new_y = 0,0
    prev_gps_dt = None
    for index in range(1, length): 
        bx, by, x, y = bind_x[index], bind_y[index], k_x[index], k_y[index]
        yaw = heading[index]	

        dx, dy = bx - x, by - y
        angle = math.atan2(dy,dx)

        dangle = angle - yaw

        servo_2 = angle_to_servo(dangle)
        servo = state_to_servo([x, y, yaw], [bx, by])

        print("delta angle: %f, state_to_servo: %f, angle_to_servo: %f" %(dangle,servo, servo_2))
            #hypotenuse = 10  # 0.5 * ((dx ** 2 + dy ** 2) ** .5)
            #USED FOR DISPLAY

            #lines.append((new_x, new_x + hypotenuse * np.cos(heading)))
            #lines.append((new_y, new_y + hypotenuse * np.sin(heading)))
            #lines.append('r')

            #x.append(new_x)
            #y.append(new_y)
        # gps_xs.append(gps_x)
        # gps_ys.append(gps_y)

    #plt.plot(*lines)
    #plt.plot(x, y)
    # plt.plot(gps_xs, gps_ys, 'p')

    #plt.show()


if __name__ == '__main__':
    print(__doc__)
    main()
