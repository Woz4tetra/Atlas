from matplotlib import pyplot as plt


class BasePlotter:
    initialized = False

    def __init__(self, num_columns, *robot_plots):
        if BasePlotter.initialized:
            raise Exception("Can't have multiple plotter instances!")
        BasePlotter.initialized = True

        self.should_skip = False

        self.closed = True

        self.robot_plots = []
        for plot in robot_plots:
            if plot.enabled:
                self.robot_plots.append(plot)

        num_plots = len(self.robot_plots)
        if num_plots < num_columns:
            num_columns = num_plots

        num_rows = num_plots // num_columns
        num_rows += num_plots % num_columns

        self.axes = {}
        self.lines = {}

        self.fig = plt.figure(0)
        self.fig.canvas.mpl_connect('close_event', lambda event: self.handle_close())

        plot_num = 1
        for plot in self.robot_plots:
            if plot.flat:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num)
                self.lines[plot.name] = self.axes[plot.name].plot([], [], **plot.properties)[0]
            else:
                self.axes[plot.name] = self.fig.add_subplot(num_rows, num_columns, plot_num, projection='3d')
                self.lines[plot.name] = self.axes[plot.name].plot([], [], [], **plot.properties)[0]

            self.axes[plot.name].set_title(plot.name)
            plot_num += 1

        self.closed = False

    def plot(self):
        return True

    def handle_close(self):
        self.close()

    def close(self):
        if not self.closed:
            self.closed = True
            plt.ioff()
            plt.gcf()
            plt.close('all')
