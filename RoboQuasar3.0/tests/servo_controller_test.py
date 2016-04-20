import math
import sys

sys.path.insert(0, "../")
from controllers.servo_map2 import state_to_servo

def test(x,y,heading,goal_x,goal_y):
    angle = state_to_servo([x,y,heading],[goal_x,goal_y])
    goal = math.atan2(goal_y - y, goal_x - x)
    print(angle, goal > 0 - goal < 0)

test(0,0,0,0,0)
test(0,0,0,10,0)
test(0,0,0,10,1)
test(0,0,0,10,-1)
test(0,0,0,100,1)
test(0,0,0,100,-1)
test(0,0,0,1000,1)
test(0,0,0,1000,-1)
