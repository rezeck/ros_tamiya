#!/usr/bin/env python
import rospy

from math import radians, cos, sin, asin, sqrt, atan2, degrees, pi, log, tan
import pid_class
import wp_manager

from ros_tamiya.msg import Gps_msg
from ros_tamiya.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import time
import os

robot = None

class RobotCar:

    def __init__(self):
        self.curr_status = 'STANDBY'
        self.WAYPOINT_RADIUS = 3.0 # [m]
        
        # Gains linear vel
        #self.KP_VEL = 300.0 
        #self.TI_VEL = 80.0
        #self.TD_VEL = 0*0.00005

        # Gains angular vel
        self.KP_PSI = 0.27
        self.TI_PSI = 80.0
        self.TD_PSI = 0*0.00007

        self.MAGNETIC_DEFLECTION = 22.2

        # Lat, lon
        self.gps_target = [0, 0] 
                            # -19.869394, -43.964293 close to the entrance of icex
                            # -19.869689, -43.964652 near
                            # -19.869767, -43.964812 verlab location 
        self.gps_pos = [0, 0]
        self.wpm = wp_manager.WPManager()

        self.orientation = 0.0
        self.speed = 0.0
        self.sat_num = 0

        self.phi = 0
        self.theta = 0
        self.psi = 0

        # Output values angular and linear
        self.out_ang = 0
        self.out_lin = 0

        self.MAX_ANGLE = 95
        
        self.MIN_LINEAR = -50
        self.MAX_LINEAR = 300

        self.pid_angle = pid_class.PID(self.KP_PSI, self.TI_PSI, self.TD_PSI, -self.MAX_ANGLE, self.MAX_ANGLE)
        #self.pid_vel   = pid_class.PID(self.KP_VEL, self.TI_VEL, self.TD_VEL, -(self.MAX_LINEAR), self.MAX_LINEAR)
        
        self.vel_publisher = rospy.Publisher('cmd_vel', Vector3, queue_size=1, latch=True)
        rospy.Subscriber("gps", Gps_msg, self.setGPS)
        rospy.Subscriber("heading", Float32, self.setHeading)
        pass

    def isGPSFixed(self):
        is_fixed = False

        if self.gps_pos[0] != 0.0 and self.gps_pos[1] != 0.0 and self.sat_num > 3:
            is_fixed = True

        return is_fixed

    def getGpsData(self):
        return {'lat': self.gps_pos[0], 'lon': self.gps_pos[1], 'speed': self.speed, 'sat_num': self.sat_num, 'orientation': self.orientation}

    def setGPS(self, data):
        #print 'setGPS data:', data.latitude, data.longitude 
        self.gps_pos = [data.latitude, data.longitude]
        self.orientation = data.orientation
        self.speed = (0.514444 * data.speed) # from knots to meters/sec
        self.sat_num = data.sat_num

    def setHeading(self, data):
        self.psi = data.data

    def setIMU(self, data):
        #print 'setIMU data:', data.heading
        self.phi = data.gyroscope
        self.theta = data.accelerometer
        self.psi = data.heading
        
        self.psi = self.psi - self.MAGNETIC_DEFLECTION
        if self.psi < 0:
		self.psi += 360
	#self.psi = self.psi % (2.0 * math.pi)

    def angularControl(self, psi_ref):
        self.pid_angle.reference(0.0)

        alpha = radians(self.psi) - radians(psi_ref)

        # garante erro entre [-pi...pi]
        while alpha <  pi:
            alpha += (2 * pi)
        while alpha > pi:
            alpha -= (2 * pi)
            
        return self.pid_angle.u(alpha)

    def speedControl(self):
        #self.pid_vel.reference(0.0)
        
        if self.wpm.isWayPointReached(self.gps_pos[0], self.gps_pos[1]) and not self.wpm.isCompleted():
            print 'WP Reached, getting next one...'
            return 300
        elif self.wpm.isCompleted():
            print 'WP completed, stopping...'
            return 300
        else:            
            rho = self.wpm.getDistanceToTarget()
            #return self.pid_vel.u(rho)

            if abs(self.out_ang) > 30:
                return 125

            control_dist = 10
            if rho > control_dist:
                return 50
            else:
                p = 100 - (rho * 100 / control_dist)
                vel = ((p/100.0) * self.MAX_LINEAR - self.MAX_LINEAR) + self.MIN_LINEAR
                if vel < self.MIN_LINEAR:
                    vel = self.MIN_LINEAR
                if vel > self.MAX_LINEAR:
                    vel = self.MAX_LINEAR

                return vel

    def calcBearingToTarget(self):
        """
        Calculate the angle between two points 
        on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians 
        lat1, lon1, lat2, lon2 = map(radians, 
            [self.gps_pos[0], self.gps_pos[1], self.gps_target[0], self.gps_target[1]])

        dLon = lon2 - lon1
        dPhi = log((tan(lat2/2.0 + pi/4.0) / tan(lat1/2.0 + pi/4.0)))
        if abs(dLon) > pi:
            if (dLon) > 0.0:
                dLon = -(2.0 * pi - dLon)
            else:
                dLon = (2.0 * pi + dLon)

        bearing = (degrees(atan2(dLon, dPhi)) + 360.0) % 360.0
        return bearing

    def haversineDistanceToTarget(self):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [self.gps_pos[0], self.gps_pos[1], self.gps_target[0], self.gps_target[1]])

        # haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        r = 6371 # Radius of earth in kilometers. Use 3956 for miles
        return (c * r) * 1000

    def control(self):
        self.gps_target[0], self.gps_target[1] = self.wpm.getCurrentWayPoint()

        self.out_ang = degrees(self.angularControl(self.calcBearingToTarget()))
        self.out_lin = self.speedControl()
        pass

    def publish(self):
        vel_msg = Vector3()        
        vel_msg.z = self.out_ang
        vel_msg.x = self.out_lin
        self.vel_publisher.publish(vel_msg)
        pass

    def printStatus(self):
        os.system('clear')
        print "Pos:", self.gps_pos
        print "Target:", self.gps_target
        print "GPS distance(m):", self.wpm.getDistanceToTarget()

        print "Bearing to target:", self.calcBearingToTarget()
        print "Curr bearing:", self.psi        

        print "Linear:", self.out_lin, 'realCommand:', abs(self.out_lin + 300) + 1000
        print "Angular:", self.out_ang, 'realCommand:', self.out_ang + 95
        pass
    
def init_current_node():
    rospy.init_node('robotcar', anonymous=True)

    robot = RobotCar()
    is_active = False

    while not rospy.is_shutdown():
        if robot.isGPSFixed() and is_active:
            robot.control()
            robot.publish()
            robot.printStatus()
        elif not robot.isGPSFixed():
            print 'Gps not fixed yet...'
            print robot.getGpsData()
        else:
            var = raw_input("The robot is ready to go, are you sure?: y/n\n")
            if var.strip() == 'y':
                is_active = True

        rospy.sleep(0.1)

if __name__ == '__main__':
    init_current_node()
