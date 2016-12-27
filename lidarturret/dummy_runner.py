from robot.interface import RobotInterface
from dummy_bot import Dummy
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class LivePlotter:
    def __init__(self, x, y, z=None):
        plt.ion()

        self.fig = plt.figure()

        # plt.axis([-500, 500, -500, 500])
        # plt.axis([0, len(xs), 0, 1000])
        self.ax = self.fig.add_subplot(111)

        self.enable_3d = True if z is not None else False

        if self.enable_3d:
            self.plot_data, = self.ax.plot(x, y, z, '-')
            self.ax.plot([0], [0], [0], 'o', color='black', markersize=10)
        else:
            self.plot_data, = self.ax.plot(x, y, '-')
            self.ax.plot(0, 0, 'o', color='black', markersize=10)

    def plot_scatter(self, x_data, y_data, z_data):
        self.plot_data.set_xdata(x_data)
        self.plot_data.set_ydata(y_data)
        self.plot_data.set_ydata(z_data)
        self.fig.canvas.draw()

        plt.pause(0.0001)


class DummyRunner(RobotInterface):
    def __init__(self):
        self.dummy = Dummy()
        # self.counter = 0

        self.accel_x = []
        self.accel_y = []
        self.accel_z = []

        self.live_plot = LivePlotter(self.accel_x, self.accel_y, self.accel_z)

        super(DummyRunner, self).__init__(self.dummy)

    def loop(self):
        if self.dummy.did_update():
            # print(self.dummy.accel_x, self.dummy.accel_y, self.dummy.accel_z)
            self.accel_x.append(self.dummy.accel_x)
            self.accel_y.append(self.dummy.accel_y)
            self.accel_z.append(self.dummy.accel_z)

            if len(self.accel_x) > 500:
                self.accel_x.pop(0)
            if len(self.accel_y) > 500:
                self.accel_y.pop(0)
            if len(self.accel_z) > 500:
                self.accel_z.pop(0)

            self.live_plot.plot_scatter(self.accel_x, self.accel_y, self.accel_z)

            # self.dummy.set_led(self.counter, not self.dummy.leds[self.counter])
            # self.counter = (self.counter + 1) % len(self.dummy.leds)


def run_dummy():
    DummyRunner().run()


run_dummy()
