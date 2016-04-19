"""
Written by Ben Warwick

servo_control.py, written for RoboQuasar3.0
Version 3/9/2015
=========

Converts the robot's current heading and a goal heading to a servo position.
"""

import math

lookup_table = [
    # servo, distance from centerline in inches
    [-73, -4.55],
    [-63, -3.95],
    [-53, -3.15],
    [-43, -2.25],
    [-33, -1.25],
    [-23, 0.0],
    [-13, 0.9],
    [3, 2.65],
    [13, 3.25],
    [23, 4.45],
    [33, 4.75],
]
hypotenuse = 50  # distance from turning point on steering
zero_index = 0
for index in range(len(lookup_table)):
    if lookup_table[index][1] == 0.0:
        zero_index = index
        break

for index in range(len(lookup_table)):
    lookup_table[index][1] = math.asin(lookup_table[index][1] / hypotenuse)


def py27_round(x):
    return int(x + math.copysign(0.5, x))


def angle_to_servo(angle):
    if angle < lookup_table[0][1]:
        return lookup_table[0][0]

    elif angle > lookup_table[-1][1]:
        return lookup_table[-1][0]
    else:
        table_index = zero_index  # index of 0 degrees
        for index in range(len(lookup_table) - 1):
            if lookup_table[index][1] <= angle <= lookup_table[index + 1][1]:
                table_index = index
                break
        # linearize between table_index and table_index + 1

        servo_val0 = lookup_table[table_index][0]
        servo_val1 = lookup_table[table_index + 1][0]
        angle_val0 = lookup_table[table_index][1]
        angle_val1 = lookup_table[table_index + 1][1]

        slope = (servo_val1 - servo_val0) / (angle_val1 - angle_val0)
        servo_angle = servo_val0 + slope * (angle - angle_val0)
        return py27_round(servo_angle)


def state_to_servo(current_state, goal_position):
    x, y, heading = current_state
    goal_x, goal_y = goal_position
    angle = math.atan2(goal_y - y, goal_x - x)
    if angle < 0:
        angle += 2 * math.pi
    angle = (angle - heading) % (2 * math.pi)
    if math.pi < angle:  # TODO: Fix wrap around problem
        angle -= 2 * math.pi
    return angle_to_servo(angle)


if __name__ == '__main__':
    def almost_equal(val1, val2, epsilon=.001):
        return abs(val1 - val2) <= epsilon


    assert (angle_to_servo(0.0) == -25)
    assert (angle_to_servo(-0.09712231328780105) == -75)
    assert (angle_to_servo(0.1072763485867491) == 35)
    assert (angle_to_servo(2.0) == 35)
    assert (angle_to_servo(-1.0) == -75)
    assert (almost_equal(angle_to_servo(0.057608353410290123), 3))
    assert (almost_equal(angle_to_servo(.00987), -21))
    assert (almost_equal(angle_to_servo(.1045), 32))
    assert (state_to_servo([0, 0, 0], [10, 0]) == -25)
    assert (state_to_servo([0, 0, 0], [0, 10]) == 35)
    assert (state_to_servo([0, 0, 0], [10, 10]) == 35)
    assert (state_to_servo([0, 0, 0], [0, -10]) == -75)
    assert (state_to_servo([0, 0, 0], [-10, 0]) == 35)
    assert (state_to_servo([4, 3, math.pi / 4], [10, 5]) == -75)
    assert (state_to_servo([4, 3, math.pi / 6], [5.67639042166, 4]) == -19)
    assert (state_to_servo([1, 2, math.pi / 8], [3, 3.0046132804729282]) == 13)
