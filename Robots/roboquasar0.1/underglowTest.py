from atlasbuggy.robot import Robot
from actuators.underglow import Underglow

import cmd

class UnderglowTest(Robot):
	def __init__ (self):
		self.underglow = Underglow()
		super(UnderglowTest, self).__init__(self.underglow)


class UnderglowCommandline(cmd.Cmd):
	def do_glow(self, line):
		glow.underglow.fancy_gradient(int(line))


glow = UnderglowTest()

