from buggypi.microcontroller.comm import *
from buggypi.microcontroller.data import *


class Robot:
    def __init__(self, sensors, commands, filter=None, joystick=None,
                 pipeline=None, capture=None, close_fn=None, log_data=True,
                 log_name=None, log_dir=None):
        self.filter = filter
        self.joystick = joystick
        self.pipeline = pipeline
        self.capture = capture

        self.close_fn = close_fn

        self.sensor_pool = SensorPool()
        self.sensors = {}
        for name, sensor_properties in sensors.items():
            sensor = Sensor(sensor_properties['sensor_id'],
                            name, sensor_properties['update_fn'],
                            sensor_properties['properties'])
            self.sensor_pool.add_sensor(sensor)
            self.sensors[name] = sensor

        self.log_data = log_data
        self.communicator = Communicator(
            self.sensor_pool, address='/dev/ttyAMA0',
            log_data=self.log_data,
            log_name=log_name,
            log_dir=log_dir)
        if not self.communicator.initialized:
            raise Exception("Communicator not initialized...")

        self.commands = {}
        for name, command_properties in commands.items():
            command_id = command_properties['command_id']
            del command_properties['command_id']
            command = Command(command_id, name, self.communicator, **command_properties)
            self.commands[name] = command

        self.time_start = time.time()

    def start(self):
        self.communicator.start()
        if self.joystick is not None:
            self.joystick.start()
        if self.capture is not None:
            self.capture.start()

    def record(self, name, value=None, **values):
        self.communicator.record(name, value, **values)

    def get_state(self):
        if self.filter is not None:
            return self.filter.state
        else:
            return None

    def close(self):
        self.commands['motors'].set(0)
        self.commands['servo'].set(0)

        if self.capture is not None:
            self.capture.stop()
        time.sleep(0.005)

        if self.joystick is not None:
            self.joystick.stop()
        time.sleep(0.005)

        self.communicator.stop()

        if self.close_fn is not None:
            self.close_fn()

    def should_stop(self):
        if self.communicator.exit_flag:
            return True
        if self.capture is not None and \
                self.capture.stopped:
            return True
        if self.joystick is not None and \
                self.joystick.exit_flag:
            return True

        return False


class RobotRunner:
    def __init__(self, robot):
        self.robot = robot

    def run(self):
        try:
            while True:
                self.main()
                if self.robot.should_stop():
                    break
        except:
            traceback.print_exc()
        finally:
            self.robot.close()

    def main(self):
        pass
