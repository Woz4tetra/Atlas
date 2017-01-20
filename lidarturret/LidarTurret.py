from breezyslam import *
from atlasbuggy.robot.robotobject import RobotObject

class LidarTurret(RobotObject):
	#Change this to desired values, need to test how they change the outcome
	def __init__(self):
		#other stuff
		self.SLAM = None
		#CHange this to test different results with different algorithms
		self.flag = SLAMObject.STATIONARY
		self.e_tick = 0
		self.total_ticks = 0
		#INT ARRAY
		self.pointcloud = []
		self.distance = 0
		super(LidarTurret, self).__init__("LidarTurret")
	def receive_first(self, packet):
		# <scan_size, scan_rate, detection_angle, distance_no_detection> (int)
		data = packet.split("\t")
		data = list(map(int,data))
		self.SLAM = SLAMObject(pybreezyslam.Laser(data),flag = self.flag)
	def receive(self, timestamp, packet):
		# <angle, distance> (int)
		data = packet.split("\t")
		self.e_tick = int(data[0])
		self.distance = int(data[1])
		self.pointcloud.append(distance)
		self.total_ticks += self.e_tick

	def updateSLAM(self,timestamp):
		if total_ticks == 661:
			self.SLAM.update(timestamp,self.pointcloud,self.e_tick)
			self.total_ticks = 0
			self.pointcloud = []


class SLAMObject (object):
	"""
	takes a breezyslam laser object and a flag to determine the
	slam algorithm that is used.
	"""
	STATIONARY = 0
	MOVING = 1
	MOVING_ODOMETRY = 2

	def __init__(self,laser, map_pixels=0,map_size =0, flag = None):
	#def __init__(self, laser, map_pixels=0, map_size=0):
		self.laser = laser
		if flag is None:
			self.flag = SLAMObject.STATIONARY
		#For odometry
		self.velocities = (0,0,0)
		self.set_algorithm()

	def set_algorithm(self):
		if self.flag == SLAMObject.STATIONARY:
			self.algorithm = breezyslam.Deterministic_SLAM(
							 self.laser,self.map_pixels,self.map_size)
		elif self.flag == SLAMObject.MOVING:
			self.algorithm =  breezyslam.RMHC_SLAM(
							  self.laser,self.map_pixels,self.map_size)
		elif self.flag == SLAMObject.MOVING_ODOMETRY:
			self.algorithm = breezyslam.CoreSLAM(
							 self.laser,self.map_pixels,self.map_size)


	def update(self, timestamp, pointcloud, theta):
		if self.flag != SLAMObject.MOVING_ODOMETRY:
			self.algorithm.update(pointcloud)
		else:
			self.update_velocities(pointcloud,tetha,timestamp)
			self.algorithm.update(pointcloud,self.velocities)

	def change_flag(self,flag):
		if flag != self.flag:
			self.flag = flag
			set_algorithm()

	def update_velocities(self,pointcloud,tetha,timestamp):
		"""
		pretty much
		estimate dxy dtetha and dt comparing with the previous measurement
		needs implementation
		"""
		pass
