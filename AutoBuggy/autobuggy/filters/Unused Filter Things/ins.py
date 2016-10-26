
class StateVector:
    """
    A class containing the vector of the state bc clarity
    """

    def __init__(self):
        self.ax = 0
        self.ay = 0
        self.vx = 0
        self.vy = 0
        self.x = 0
        self.y = 0
        self.w = 0
        self.yaw = 0


class INS:
    """
    This INS class takes in IMU data to form a view of the system.
    It uses pure kinematics models using integration.

    The IMU inputs are ax, ay, w, yaw

    The INS keeps track of ax, ay, vx, vy, x, y, w, and yaw

    ax = (ax + ax')/2
    ay = (ay + ay')/2
    vx = ax * dt + vx
    vy = ay * dt + vy
    x  = vx * dt + x
    y  = vy * dt + y

    w   = (w + w')/2
    yaw = w *dt + yaw

    The INS does not update itself completely every timestep.
    Rather, it takes in self.max data sets,
    then averages them and uses that to update its state.
    """

    def __init__(self, timestamp, num_inputs):
        self.max = num_inputs  # max number of datasets

        self.state = StateVector()
        self.temp = []  # this will hold the 10 interim state vecotrs

        # this will be used to calculate the time needed for dt
        self.time = timestamp

    def update(self, ax, ay, yaw, timestamp):
        """
        NOTE: YAW IS NOT REALLY USED HERE
        SO
        lets just ignore the w value and let the imu use its own sensor fusion
        to give us the yaw which has always seemed really good.
        and simplify the INS
        """
        self.temp.append([ax, ay, yaw])  # add it to temp
        length = len(self.temp)  # get length of temp bc faster to do once
        if length >= self.max:  # only need to do more if there are 10 already
            temp_ax = 0  # create temp variables
            temp_ay = 0
            # temp_w = 0
            temp_yaw = 0
            for i in range(length):  # sum up all of them
                # could be made parallel but idk how in
                # python
                temp_ax += self.temp[i][0]
                temp_ay += self.temp[i][1]
                temp_yaw += self.temp[i][2]

            avg_ax = temp_ax / length  # now get the averages
            avg_ay = temp_ay / length
            avg_yaw = temp_yaw / length

            # used state ax and state ay and state w for consistency and
            # because it makes sense

            # calculate time during which the imu entries were averaged
            dt = timestamp - self.time

            # now update the thing
            self.state.ax = float((avg_ax + self.state.ax) / 2.0)

            # averaging these so the past is used in some fashion
            # (prevents explosions a bit)
            self.state.ay = float((avg_ay + self.state.ay) / 2.0)
            self.state.vx += self.state.ax * dt
            self.state.vy += self.state.ay * dt
            self.state.x = (self.state.x + self.state.vx * dt +
                            0.5 * self.state.ax * dt * dt)
            self.state.y = (self.state.y + self.state.vy * dt +
                            0.5 * self.state.ay * dt * dt)
            self.state.yaw = float((avg_yaw + self.state.yaw) / 2.0)

            # now we need to reset things
            self.temp = []  # reinitialize temp to be empty
            self.time = timestamp  # reinitialized time

    def __str__(self):
        return (
            "ax = %2.2f, ay = %2.2f, vx = %2.2f, vy = %2.2f, x = %2.2f, y = "
            "%2.2f, yaw = %2.2f" % (
                self.state.ax, self.state.ay, self.state.vx, self.state.vy,
                self.state.x, self.state.y, self.state.yaw))


# TODO: change to make it where it uses an inputted t, ignoring it
# TODO: unless its used (every 10)
# TODO: figure out how to reconcile the starting position with the
# TODO: absolute position of ned (north east down) RESEARCH

if __name__ == '__main__':
    def run_10_updates(ins, ax, ay, yaw, time):
        for t in range(10):
            ins.update(ax, ay, yaw, time + t)
        print(ins)

    def test_all():
        test_ins = INS(0)
        # dummy, conceivable data for ax, ay
        run_10_updates(test_ins, 0, 0, 0, 0)
        run_10_updates(test_ins, 1, 1, 0, 10)
        run_10_updates(test_ins, 1, 1, 0, 20)
        run_10_updates(test_ins, 0, 0, 0, 30)

        test_ins = INS(0)
        # dummy, conceivable data for w, yaw
        run_10_updates(test_ins, 0, 0, 1, 0)
        run_10_updates(test_ins, 0, 0, 1, 10)
        run_10_updates(test_ins, 0, 0, 5, 0)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)
        run_10_updates(test_ins, 0, 0, 10, 10)

    test_all()
