import math

def map_servo(angle):
    return 0


def servo_value(current_state, goal_position):
    x, y, heading = current_state
    goal_x, goal_y = goal_position
    theta = math.atan2(goal_y - y, goal_x - x)
    return map_servo(theta - heading)
