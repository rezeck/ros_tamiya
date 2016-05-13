#!/usr/bin/env python
#############################################################################
# ROS GPS driver for Tamiya (GPS Ublox LEA-6T-0)
# By Paulo Rezeck
# rezeck@dcc.ufmg.br
# Computer Vision and Robotic Laboratory (VeRlab/UFMG)
# Note: This code has been modified from the original developed 
# by Armando Alver Neto for robotovero. 
#############################################################################
import os, serial, sys, threading
import math

import rospy
from ros_tamiya.msg import Gps_msg
#############################################################################


#############################################################################
# class GPS
#############################################################################
class GPS:
	#########################################################################
	# construtor
	#########################################################################
	def __init__(self):
		#####################################################################
		# Getting serial port name
		port = rospy.get_param('port', '/dev/ttyUSB0')
		baud = rospy.get_param('baud', 19200)

		# Open and Setup the serial port
		try:
			self._device = serial.Serial(port, baud)
		except:
			rospy.loginfo('Serial Port Error: could not get ' + port)
			sys.exit(0)

		rospy.loginfo('GPS Serial Port ' + port + ' is open!')
		
		# Cleaning the buffer
		self._device.flushInput()
		self._device.flushOutput()
		buff = self._device.read(self._device.inWaiting())
		
		#####################################################################
		# members
		self._UTC = 0
		self._latitude = 0.0
		self._north_south = ''
		self._longitude = 0.0
		self._west_east = ''
		self._fix_quality = 0
		self._nsat = 0
		self._hdop = 0
		self._altitude = 0.0
		self._hgeoid = 0.0
		self._fix3d = 1
		self._status = 'V'
		self._speed = 0.0
		self._orientation = 0.0
		self._data = '130412'
		
		self._x = 0.0
		self._y = 0.0
		self._utmzone = 0
		
		#####################################################################
		# critical section object
		self.lock = threading.Lock()
		
		#####################################################################
		# Init thread to capture datas from GPS
		self.th_getdata = threading.Thread(target=self.getData, args = ( ) )
		self.th_getdata.daemon = True # causes the thread to terminate when the main process ends
		self.th_getdata.start()
		
	#########################################################################
	# Read data from GPS
	#########################################################################
	def getData(self):
		# incrementa prioridade da thread (talvez seja desnecessario)
		os.nice(10)
		
		# colect data loop
		while True:
			# read one gps line
			line = self._device.readline()
			#print line
			
			# enter critical section
			self.gpsAcquire()
			
			# try parse message
			try:
				self.parseMsg(line)
			except:
				#print 'GPS msg error'
				None
			
			# leave critical section
			self.gpsRelease()
		
	#########################################################################
	# Make a parser from GPS messages
	#########################################################################
	def parseMsg (self, msg):
	
		# separate fields
		msg = msg.split(",")
		
		# message type
		#########################
		if msg[0] == '$GPGGA':
			self.parseGGA(msg)
		#########################
		elif msg[0] == '$GPRMC':
			self.parseRMC(msg)
		#########################
		elif msg[0] == '$GPGSA':
			self.parseGSA(msg)
		#########################
		elif msg[0] == '$GPGLL':
			self.parseGLL(msg)
		#########################
		elif msg[0] == '$GPGSV':
			None
		#########################	
		elif msg[0] == '$GPZDA':
			None
		#########################
		elif msg[0] == '$GPVTG':
			None
		#########################
		else:
			None
		
		# After left NMEA message, clean buffer
		if msg[0] == '$GPGLL':
			buff = self._device.read(self._device.inWaiting())
				
	#########################################################################
	''' $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47

	Where:
		GGA          Global Positioning System Fix Data
		123519       Fix taken at 12:35:19 UTC
		4807.038,N   Latitude 48 deg 07.038' N
		01131.000,E  Longitude 11 deg 31.000' E
		1            Fix quality: 0 = invalid
                                  1 = GPS fix (SPS)
                                  2 = DGPS fix
                                  3 = PPS fix
			                      4 = Real Time Kinematic
			                      5 = Float RTK
                                  6 = estimated (dead reckoning) (2.3 feature)
			                      7 = Manual input mode
			                      8 = Simulation mode
		08           Number of satellites being tracked
		0.9          Horizontal dilution of position
		545.4,M      Altitude, Meters, above mean sea level
		46.9,M       Height of geoid (mean sea level) above WGS84 ellipsoid
		(empty field) time in seconds since last DGPS update
		(empty field) DGPS station ID number
		*47          the checksum data, always begins with * '''
	#########################################################################
	def parseGGA(self, msg):
	
		self._UTC = msg[1]
		
		# Update position
		self.calcPosition(msg[2], msg[4], msg[3], msg[5])
		
		self._fix_quality = int(msg[6])
		self._nsat = int(msg[7])
		#self._hdop		= float(msg[8])
		#self._altitude	= float(msg[9])
		#self._hgeoid	= float(msg[11])
		
	#########################################################################
	''' $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A

	Where:
		RMC          Recommended Minimum sentence C
		123519       Fix taken at 12:35:19 UTC
		A            Status A=active or V=Void.
		4807.038,N   Latitude 48 deg 07.038' N
		01131.000,E  Longitude 11 deg 31.000' E
		022.4        Speed over the ground in knots
		084.4        Track angle in degrees True
		230394       Date - 23rd of March 1994
		003.1,W      Magnetic Variation
		*6A          The checksum data, always begins with * '''
	#########################################################################
	def parseRMC(self, msg):
		self._UTC = msg[1]
		self._status = msg[2]
		
		# Update position
		self.calcPosition(msg[3], msg[5], msg[4], msg[6])
		
		self._speed = float(msg[7])
		self._orientation = float(msg[8])
		#self._data = msg[9]
	
	#########################################################################
	''' $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39

	Where:
		GSA      Satellite status
		A        Auto selection of 2D or 3D fix (M = manual) 
		3        3D fix - values include: 1 = no fix
                                          2 = 2D fix
                                          3 = 3D fix
		04,05... PRNs of satellites used for fix (space for 12) 
		2.5      PDOP (dilution of precision) 
		1.3      Horizontal dilution of precision (HDOP) 
		2.1      Vertical dilution of precision (VDOP)
		*39      the checksum data, always begins with * '''
	#########################################################################
	def parseGSA(self, msg):
		self._fix3d = int(msg[2])
	
	#########################################################################
	'''$GPGLL,4916.45,N,12311.12,W,225444,A,*1D

	Where:
    	GLL          Geographic position, Latitude and Longitude
    	4916.46,N    Latitude 49 deg. 16.45 min. North
    	12311.12,W   Longitude 123 deg. 11.12 min. West
    	225444       Fix taken at 22:54:44 UTC
    	A            Data Active or V (void)
    	*iD          checksum data '''
	#########################################################################
	def parseGLL(self, msg):
	
		# atualiza a posicao
		self.calcPosition(msg[1], msg[3], msg[2], msg[4])
		
		# status
		self._status = msg[6]
		
	#########################################################################
	''' $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48

	where:
		VTG          Track made good and ground speed
		054.7,T      True track made good (degrees)
		034.4,M      Magnetic track made good
		005.5,N      Ground speed, knots
		010.2,K      Ground speed, Kilometers per hour
		*48          Checksum '''
	#########################################################################
	def parseVTG(self, msg):
		if (len(msg[5]) > 0) and (msg[6] == 'N'):
			self._speed = float(msg[7])
	
	#########################################################################
	# Digital format (dddd.mmm) 2 meters(m)
	#########################################################################
	def wgs2utm(self, Lat, Lon):
	
		# coordinates in radians
		lat = Lat*(math.pi/180.0)
		lon = Lon*(math.pi/180.0)
		
		# WGS84 parameters
		a = 6378137 			#semi-major axis
		b = 6356752.314245 		#semi-minor axis
		e = math.sqrt(1 - pow(b/a, 2)) 	#eccentricity
		e2 = pow(e, 2)
		e4 = pow(e, 4)
		e6 = pow(e, 6)
		
		# UTM parameters
		# lat0 = 0 						#reference latitude, not used here
		Lon0 = math.floor(Lon/6)*6 + 3 	#reference longitude in degrees
		lon0 = Lon0*(math.pi/180.0)		#in radians
		k0 = 0.9996 					#scale on central meridian
		
		FE = 500000 			#false easting
		FN = (Lat < 0)*10000000 #false northing
		
		# Equations parameters
		eps = e2/(1 - e2) # e prime square
		# N: radius of curvature of the earth perpendicular to meridian plane
		# Also, distance from point to polar axis
		N = a/math.sqrt(1 - e2*pow(math.sin(lat),2))
		T = pow(math.tan(lat),2)
		C = ((e2)/(1-e2))*pow(math.cos(lat), 2)
		A = (lon-lon0)*math.cos(lat)
		
		# M: true distance along the central meridian from the equator to lat
		M = a*( +(1- e2/4 - 3*e4/64 - 5*e6/256 )*lat
				-(3*e2/8 + 3*e4/32 + 45*e6/1024 )*math.sin(2*lat)
				+(15*e4/256 + 45*e6/1024 )*math.sin(4*lat)
				-(35*e6/3072 )*math.sin(6*lat) )
				
		# easting
		x = FE + k0*N*( A+(1-T+C)*pow(A,3)/6 + ( 5-18*T+pow(T,2)+72*C-58*eps )*pow(A,5)/120 )
		
		# northing
		# M(lat0) = 0 so not used in following formula
		y = FN + k0*M + k0*N*math.tan(lat)*((pow(A,2)/2) 
									+ (5 - T + 9*C + 4*pow(C,2))*(pow(A,4)/24) 
									+ (61 - 58*T + pow(T,2) + 600*C - 330*eps)*pow(A,6)/720)
		
		# UTM zone
		utmzone = int(math.floor(Lon0/6) + 31)
		
		return round(x,4), round(y,4), utmzone
		
	#########################################################################
	# WGS84 to digital format (dddd.mmm) 2 +-(dd.mmmmm)
	#########################################################################
	def wgs2dig(self, wgs, s):
		deg = int(math.floor(wgs/100.0))
		degpos = deg + (( wgs - (deg*100.0) )/60.0)
        
		# if South or West
		if (s == 'S') or (s == 'W'):
			degpos = -degpos
		
		# apenas 5 casas de precisao
		return round(degpos, 10)
	#########################################################################
	# update the lat, lon, x and y position
	#########################################################################
	def calcPosition(self, latmsg, lonmsg, nsmsg, wemsg):
		
		# aux variables
		lat = self.wgs2dig(float(latmsg), nsmsg)
		lon = self.wgs2dig(float(lonmsg), wemsg)

		# latitude
		self._latitude = lat
		self._north_south = nsmsg
		
		# longitude
		self._longitude = lon
		self._west_east = wemsg
		
		# converte (lat,lon) em formato digital para metros
		self._x, self._y, self._utmzone = self.wgs2utm(lat, lon)
		
		return

	#########################################################################
	# Getting message
	#########################################################################
	def getMessage(self):
		# enter critical section
		self.gpsAcquire()
		self.gps_msg = Gps_msg()
		self.gps_msg.header.stamp = rospy.Time.now()
		self.gps_msg.header.frame_id = "gps"

		# GPS fix quality
		if self._fix_quality == 0:
			self.gps_msg.fix_quality = "GPS invalid"
		elif self._fix_quality == 1:
			self.gps_msg.fix_quality = "GPS fix (SPS)"
		elif self._fix_quality == 2:
			self.gps_msg.fix_quality = "DGPS fix"
		elif self._fix_quality == 3:
			self.gps_msg.fix_quality = "PPS fix"
		elif self._fix_quality == 4:
			self.gps_msg.fix_quality = "Real Time Kinematic"
		elif self._fix_quality == 5:
			self.gps_msg.fix_quality = "Float RTK"
		elif self._fix_quality == 6:
			self.gps_msg.fix_quality = "Estimated (dead reckoning)"
		elif self._fix_quality == 7:
			self.gps_msg.fix_quality = "Manual input mode"
		elif self._fix_quality == 8:
			self.gps_msg.fix_quality = "Simulation mode"
		else:
			self.gps_msg.fix_quality = "UNDEFINED MODE"

		# GPS 3D FIX
		if self._fix3d == 1:
			self.gps_msg.fix3d = 'no fix'
		elif self._fix3d == 2:
			self.gps_msg.fix3d = '2D fix'
		elif self._fix3d == 3:
			self.gps_msg.fix3d = '3D fix'
		else:
			self.gps_msg.fix3d = 'UNDEFINED FIX'
				
		# Status
		if self._status == 'A':
			self.gps_msg.status = 'Active'
		elif self._status == 'V':
			self.gps_msg.status = 'Void'
		else:
			self.gps_msg.status = 'UNDEFINED STATUS'
		
		self.gps_msg.sat_num = self._nsat
		self.gps_msg.latitude = self._latitude
		self.gps_msg.longitude = self._longitude
		self.gps_msg.utmzone = self._utmzone
		self.gps_msg.X = self._x
		self.gps_msg.Y = self._y
		self.gps_msg.speed = self._speed
		self.gps_msg.orientation = self._orientation
		
		# leave critical section
		self.gpsRelease()

	#########################################################################
	# Show data
	#########################################################################
	def show(self):
		# enter critical section
		self.gpsAcquire()
			
		print '-------------------------------------------------------------'
		print 'GPS data:'
		
		aux = '\tQuality: '
		# Print GPS fix quality
		if self._fix_quality == 0:
			print aux + 'GPS invalid'
		elif self._fix_quality == 1:
			print aux + 'GPS fix (SPS)'
		elif self._fix_quality == 2:
			print aux + 'DGPS fix'
		elif self._fix_quality == 3:
			print aux + 'PPS fix'
		elif self._fix_quality == 4:
			print aux + 'Real Time Kinematic'
		elif self._fix_quality == 5:
			print aux + 'Float RTK'
		elif self._fix_quality == 6:
			print aux + 'Estimated (dead reckoning)'
		elif self._fix_quality == 7:
			print aux + 'Manual input mode'
		elif self._fix_quality == 8:
			print aux + 'Simulation mode'
		else:
			print aux + 'UNDEFINED MODE'

		aux = '\t3D fix: '
		# Print GPS 3D FIX
		if self._fix3d == 1:
			print aux + 'no fix'
		elif self._fix3d == 2:
			print aux + '2D fix'
		elif self._fix3d == 3:
			print aux + '3D fix'
		else:
			print 'UNDEFINED FIX'
				
		# Print status
		aux = '\tStatus: '
		if self._status == 'A':
			print aux + 'Active'
		elif self._status == 'V':
			print aux + 'Void'
		else:
			print 'UNDEFINED STATUS'
		
		print 'No sats:', self._nsat
		print 'Latitude: ', '(' + self._north_south + ')\t', self._latitude 
		print 'Longitude:', '(' + self._west_east   + ')\t', self._longitude
		print 'X[m]: ', self._x, '\tUTM zone ', + self._utmzone
		print 'Y[m]: ', self._y
		print 'Speed:', self._speed
		print 'Orientation:', self._orientation
		print '-------------------------------------------------------------'
		
		# leave critical section
		self.gpsRelease()
	
	#########################################################################
	# get lock
	#########################################################################
	def gpsAcquire(self):
		self.lock.acquire()
		
	#########################################################################
	# leave lock
	#########################################################################
	def gpsRelease(self):
		self.lock.release()
	
	#########################################################################
	# destrutor
	#########################################################################
	def __del__(self):
		self._device.close()

#############################################################################
# MAIN
#############################################################################
import time
if __name__=="__main__":
	rospy.init_node('tamiya_gps', anonymous=False)
	rospy.loginfo('Initializing ros tamiya gps')
	gps = GPS()
	pub = rospy.Publisher('gps', Gps_msg, queue_size=1)
	rate = rospy.Rate(2) # 10hz

	while not rospy.is_shutdown():
		gps.getMessage()
		rospy.loginfo('Reading')
  		pub.publish(gps.gps_msg)
  		rate.sleep()
