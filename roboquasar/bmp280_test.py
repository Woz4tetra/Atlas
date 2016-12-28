from atlasbuggy.robot import Robot
from joysticks.gc_joystick import GCjoystick


class BmpRobot(Robot):
    def __init__(self):
        sensors = dict(
            bmp=dict(
                sensor_id=0, properties=['altitude', 'temperature', 'pressure'],
                update_fn=lambda: self.print_sensor()
            )
        )

        commands = dict(
            leds=dict(command_array={
                "red": 0,
                "yellow": 1,
                "green": 2,
            }, range=(0, 2),
                mapping={
                    "off": 0,
                    "on": 1,
                    "toggle": 2
                }),
            blue_led=dict(command_id=3, range=(0, 255), mapping={
                "off": 0,
                "on": 255,
            })
        )

        joystick = GCjoystick(
            button_down_fn=lambda button, params: self.button_dn(button),
            button_up_fn=lambda button, params: self.button_up(button),
            axis_active_fn=lambda axis, value, params: self.axis_active(
                axis, value),
            axis_inactive_fn=lambda axis, params: self.axis_inactive(axis)
        )

        super(BmpRobot, self).__init__(
            sensors, commands, '/dev/tty.usbserial',
            ['tty.usbmodem*', 'cu.usbmodem*'],
            joystick=joystick, close_fn=self.close_fn, log_data=True,
            log_dir="Pressure Tests")

        self.bmp = self.sensors['bmp']

        self.red = self.commands['leds']['red']
        self.yellow = self.commands['leds']['yellow']
        self.green = self.commands['leds']['green']
        self.blue = self.commands['blue_led']

    def button_dn(self, button):
        if button == 'A':
            self.red.set('on')
        elif button == 'B':
            self.yellow.set('on')
        elif button == 'X':
            self.green.set('on')
        elif button == 'Y':
            self.blue.set('on')

    def button_up(self, button):
        if button == 'A':
            self.red.set('off')
        elif button == 'B':
            self.yellow.set('off')
        elif button == 'X':
            self.green.set('off')
        elif button == 'Y':
            self.blue.set('off')

    def axis_active(self, axis, value):
        if axis == "left y":
            self.blue.set(int(abs(value) * 255))

    def axis_inactive(self, axis):
        if axis == "left y":
            self.blue.set('off')

    def print_sensor(self):
        print("%0.4f m, %0.4fC, %0.4f Pa" % self.bmp.get(all=True), end='\r')

    def close_fn(self):
        print("Pressure Test closing")


BmpRobot().run()
