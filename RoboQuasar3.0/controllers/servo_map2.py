import math

lookup_table = [
    # servo, distance from centerline in inches
    #TODO CHECK THIS SHIT
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


adjacent = 50  # distance from turning point on steering
zero_index = 0
for index in range(len(lookup_table)):
    if lookup_table[index][1] == 0.0:
        zero_index = index
        break

for index in range(len(lookup_table)):
    lookup_table[index][1] *= -1
    lookup_table[index][1] = math.atan2(lookup_table[index][1], adjacent)

print(lookup_table)

def angle_to_servo(angle):
    if angle > lookup_table[0][1]:
        return lookup_table[0][0]
    elif angle < lookup_table[-1][1]:
        return lookup_table[-1][0]

    table_index = 0
    for index in range(len(lookup_table) - 1):
        if (angle == lookup_table[index][1]):
            return lookup_table[index][0]
        if ((angle < lookup_table[index][1]) and
            (angle > lookup_table[index+1][1])):
            table_index = index

    servo_val0 = lookup_table[table_index][0]
    servo_val1 = lookup_table[table_index + 1][0]
    angle_val0 = lookup_table[table_index][1]
    angle_val1 = lookup_table[table_index + 1][1]

    slope = (servo_val1 - servo_val0) / (angle_val1 - angle_val0)
    servo_angle = servo_val0 + slope * (angle - angle_val0)
    return round(servo_angle)


def state_to_servo(current_state, goal_state):
    x, y, yaw = current_state
    goal_x, goal_y = goal_state

    goal_angle = math.atan2(goal_y-y, goal_x-x)

    if goal_angle < 0:
        goal_angle += 2 * math.pi
    dtheta = (goal_angle - yaw) % (2 * math.pi)
    if math.pi < dtheta:  # TODO: Fix wrap around problem
        dtheta -= 2 * math.pi

    return angle_to_servo(dtheta)
