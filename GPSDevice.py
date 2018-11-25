from gps3 import agps3
#import KONST
#import multitasking
#import signal
# kill all tasks on ctrl-c

class GPSDevice():
	#signal.signal(signal.SIGINT, multitasking.killall)

	gps_socket = agps3.GPSDSocket()
	data_stream = agps3.DataStream()
	gps_socket.connect()
	gps_socket.watch()
	alt = 0.0
	lat = 0.0
	lon = 0.0
	time = u'2018-10-02T14:55:05.000Z'
	speed = 0.0
	cours = 0.0
	climb = 0.0
	status = 0.0
	latErr = 0.0
	lonErr = 0.0
	altErr = 0.0
	courseErr = 0.0
	speedErr = 0.0
	timeOffset = 0.0
	gdoppdop = 0.0
	tdop = 0.0
	ydop = 0.0
	xdop = 0.0
	vdop = 0.0
	hdop = 0.0

	def StartRead(self):
		self.gps_socket = agps3.GPSDSocket()
		self.data_stream = agps3.DataStream()
		self.gps_socket.connect()
		self.gps_socket.watch()

	def ReadOnce(self, new_data):
		self.data_stream.unpack(new_data)
		self.alt = self.data_stream.alt
		self.lat = self.data_stream.lat
		self.lon = self.data_stream.lon
		self.time = self.data_stream.time
		self.speed = self.data_stream.speed
		self.cours = self.data_stream.track
		self.climb = self.data_stream.climb
		self.status = self.data_stream.mode
		self.latErr = self.data_stream.epx
		self.lonErr = self.data_stream.epy
		self.altErr = self.data_stream.epv
		self.courseErr = self.data_stream.epc
		self.speedErr = self.data_stream.eps
		self.timeOffset = self.data_stream.ept
		self.gdop = self.data_stream.gdop
		self.pdop = self.data_stream.pdop
		self.tdop = self.data_stream.tdop
		self.ydop = self.data_stream.ydop
		self.xdop = self.data_stream.xdop
		self.vdop = self.data_stream.vdop
		self.hdop = self.data_stream.hdop

	#@multitasking.task
	def read(self):
		for new_data in self.gps_socket:
			if new_data:
				self.data_stream.unpack(new_data)
				self.alt = self.data_stream.alt
				self.lat = self.data_stream.lat
				self.lon = self.data_stream.lon
				self.time = self.data_stream.time
				self.speed = self.data_stream.speed
				self.cours = self.data_stream.track
				self.climb = self.data_stream.climb
				self.status = self.data_stream.mode
				self.latErr = self.data_stream.epx
				self.lonErr = self.data_stream.epy
				self.altErr = self.data_stream.epv
				self.courseErr = self.data_stream.epc
				self.speedErr = self.data_stream.eps
				self.timeOffset = self.data_stream.ept
				self.gdop = self.data_stream.gdop
				self.pdop = self.data_stream.pdop
				self.tdop = self.data_stream.tdop
				self.ydop = self.data_stream.ydop
				self.xdop = self.data_stream.xdop
				self.vdop = self.data_stream.vdop
				self.hdop = self.data_stream.hdop
				#print('Altitude = ', self.data_stream.alt)
				#print('Lat = ', self.data_stream.lat)
				#print('Lon = ', self.data_stream.lon)
				#print('Time = ', self.data_stream.time)
				#print('speed = ', self.data_stream.speed)
				#print('cours = ', self.data_stream.track)

	def ReadWithq(self, q_gps):
		for new_data in self.gps_socket:
			if new_data:
				self.data_stream.unpack(new_data)
				self.alt = self.data_stream.alt
				self.lat = self.data_stream.lat
				self.lon = self.data_stream.lon
				self.time = self.data_stream.time
				self.speed = self.data_stream.speed
				self.cours = self.data_stream.track
				self.climb = self.data_stream.climb
				self.status = self.data_stream.mode
				self.latErr = self.data_stream.epx
				self.lonErr = self.data_stream.epy
				self.altErr = self.data_stream.epv
				self.courseErr = self.data_stream.epc
				self.speedErr = self.data_stream.eps
				self.timeOffset = self.data_stream.ept
				self.gdop = self.data_stream.gdop
				self.pdop = self.data_stream.pdop
				self.tdop = self.data_stream.tdop
				self.ydop = self.data_stream.ydop
				self.xdop = self.data_stream.xdop
				self.vdop = self.data_stream.vdop
				self.hdop = self.data_stream.hdop
				#print('Altitude = ', self.data_stream.alt)
				#print('Lat = ', self.data_stream.lat)
				#print('Lon = ', self.data_stream.lon)
				#print('Time = ', self.data_stream.time)
				if(q_gps.empty()):
					q_gps.put(self)
                    

	def GetAlt(self):
		return self.alt

	def GetLat(self):
		return self.lat

	def GetLon(self):
		return self.lon

	def GetTime(self):
		return self.time

	def GetSpeed(self):
		return self.speed

	def GetCours(self):
		return self.cours

	def GetClimb(self):
		return self.climb
    



def Process(q_gps):
    gps = GPSDevice()
    print ("In Process GPS")
    gps.ReadWithq(q_gps)
    #for new_data in gps.gps_socket:
    #    if (new_data):
    #        gps.ReadOnce(new_data)
    #    q_gps.put(gps)
