from atlasbuggy.robot.interface import RobotInterface
#from joysticks.logitech import Logitech
from LidarTurret import LidarTurret

class LidarRunner(RobotInterface):
    def __init__(self):
    	self.LidarTurret = LidarTurret()

    	super(LidarRunner, self).__init__(
    		self.LidarTurret,
    		#joystick=Logitech(),
    		log_data=False,
    		#debug prints=True,
    	)

	def packet_received(self, timestamp, whoiam, packet):
		if self.did_receive(self.LidarTurret) # replace dummy with something else
			# breezy slam code
			# make point cloud and pass directly into breezy slam code


def run_lidar():
	LidarRunner.run()

run_lidar()