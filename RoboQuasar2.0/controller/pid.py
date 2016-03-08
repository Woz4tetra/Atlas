# contains the sensor pid controller. Uses the "trailer hitch" principle.
# It's like a spring is pulling on the end of trailer hitch towards equilibrium

class PID():
    def __init__(self):
        pass

    def update(self, measured, goal):
        x, y, heading = measured
        goal_x, goal_y = goal


