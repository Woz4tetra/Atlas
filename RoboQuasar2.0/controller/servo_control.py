import math

lookup_table = [
    [-0.09712231328780105, -75],
    [-0.09103477803741508, -70],
    [-0.08495062388569469, -65],
    [-0.07684327962364361, -60],
    [-0.07076612019860314, -55],
    [-0.06064322403151619, -50],
    [-0.052549434564823666, -45],
    [-0.04445908938177131, -40],
    [-0.030307669966186872, -35],
    [-0.018182820083936495, -30],
    [0.0, -25],  # default index
    [0.012121508956561831, -20],
    [0.020203394601319605, -15],
    [0.030307669966186872, -10],
    [0.04041504166264684, -5],
    [0.052549434564823666, 0],
    [0.06266727225575658, 5],
    [0.0687409931432045, 10],
    [0.07684327962364361, 15],
    [0.08495062388569469, 20],
    [0.09306356936250755, 25],
    [0.10321346126387099, 30],
    [0.1072763485867491, 35],
]

def map_servo(angle):
    if angle < lookup_table[0][0]:
        return lookup_table[0][1]

    elif angle > lookup_table[22][0]:
        return lookup_table[22][1]
    else:
        table_index = 10  # index of 0 degrees
        for index in range(len(lookup_table) - 1):
            if lookup_table[index][0] <= angle <= lookup_table[index + 1][0]:
                table_index = index
                break
        # linearize between table_index and table_index + 1

        slope = (lookup_table[table_index+1][1] - lookup_table[table_index][1])/(
            lookup_table[table_index+1][0]-lookup_table[table_index][0])
        servo_angle = lookup_table[table_index][1] + slope * (angle - lookup_table[table_index][0])
        return round(servo_angle)

# TODO: Fix to be oriented in the correct axis
def servo_value(current_state, goal_position):
    x, y, heading = current_state
    goal_x, goal_y = goal_position
    return map_servo(math.atan2(goal_y - y, goal_x - x) - heading)

if __name__ == '__main__':
    def almost_equal(val1, val2, epsilon = .001):
        return abs(val1-val2) <= epsilon
    assert(map_servo(0.0) == -25)
    assert(map_servo(-0.09712231328780105) == -75)
    assert(map_servo(0.1072763485867491) == 35)
    assert(map_servo(2.0) == 35)
    assert(map_servo(-1.0) == -75)
    assert(almost_equal(map_servo(0.057608353410290123), 2))
    assert(almost_equal(map_servo(.00987), -21))
    assert(almost_equal(map_servo(.1045), 32))

