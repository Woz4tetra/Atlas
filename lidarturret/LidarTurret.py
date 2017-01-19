class LidarTurret(RobotOject):
	def __init__(self):
		#other stuff
		self.scan_size = 0
		self.scan_rate = 0
		self.detection_angle = 0
		self.distance_no_detection = 0
		self.e_tick = 0
		self.distance = 0
		super(LidarTurret, self).__init__("LidarTurret")
	def receive_first(self, packet):
		# <scan_size, scan_rate, detection_angle, distance_no_detection> (int)
		data = packet.split("\t")
		self.scan_size = int(data[0])
		self.scan_rate = int(data[1])
		self.detection_angle = int(data[2])
		self.distance_no_detection = int(data[3])
	def receive(self, timestamp, packet):
		# <angle, distance> (int)	
		data = packet.split("\t")
		self.e_tick = int(data[0])
		self.distance = int(data[1])
		