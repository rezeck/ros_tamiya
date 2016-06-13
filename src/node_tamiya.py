#!/usr/bin/env python
import rospy

from math import radians, cos, sin, asin, sqrt, atan2, degrees, pi, log, tan
import pid_class

from ros_tamiya.msg import Gps_msg
from ros_tamiya.msg import Imu
from geometry_msgs.msg import Twist
import time

robot = None

class RobotCar:
    def __init__(self):
        self.WAYPOINT_RADIUS = 2.0 # [m]
        
        # Gains linear vel
        self.KP_VEL = 5.0 
        self.TI_VEL = 80.0
        self.TD_VEL = 0*0.00005

        # Gains angular vel
        self.KP_PSI = 0.7
        self.TI_PSI = 70.0
        self.TD_PSI = 0*0.00007

        self.MAGNETIC_DEFLECTION = 123

        # Lat, lon
        self.gps_target = [-19.869689, -43.964652] 
                            # -19.869394, -43.964293 close to the entrance of icex
                            # -19.869689, -43.964652 near 
        self.gps_pos = [-19.869951, -43.965057]

        self.orientation = 0
        self.speed = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0

        # Output values angular and linear
        self.out_ang = 0
        self.out_lin = 0

        self.MAX_ANGLE = 95
        self.MAX_LINEAR = 300

        self.pid_angle = pid_class.PID(self.KP_PSI, self.TI_PSI, self.TD_PSI, -self.MAX_ANGLE, self.MAX_ANGLE)
        self.pid_vel   = pid_class.PID(self.KP_VEL, self.TI_VEL, self.TD_VEL, -self.MAX_LINEAR, self.MAX_LINEAR)
        
        self.vel_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1, latch=True)
        rospy.Subscriber("gps", Gps_msg, self.setGPS)
        rospy.Subscriber("imu", Imu, self.setIMU)
        pass

    def setGPS(self, data):
        #print 'setGPS data:', data.latitude, data.longitude 
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.orientation = data.orientation
        self.speed = (0.514444 * data.speed) # from knots to meters/sec

    def setIMU(self, data):
        #print 'setIMU data:', data.heading
        self.phi = data.gyroscope
        self.theta = data.accelerometer
        self.psi = data.heading
        
        # self.psi = (atan2(data.magnetometer.y, data.magnetometer.x) * 180) / pi
        # if self.psi < 0:
        #     self.psi += 360
        #self.psi = self.psi + self.MAGNETIC_DEFLECTION
        #self.psi = self.psi % (2.0 * math.pi)

    def set_linear_vel(self, vel):
        self.lin_vel = vel
        pass

    def set_angular_vel(self, vel):
        self.ang_vel = vel
        pass

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
        self.pid_vel.reference(0.0)
        
        rho = self.haversineDistanceToTarget()

        print "GPS distance(m):", rho 
        
        if rho <= self.WAYPOINT_RADIUS:
            print "REACHED!!!, get new WP"
            return 1600
        
        return self.pid_vel.u(rho)

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
        print "Compass:", self.psi

        bearing = self.calcBearingToTarget()
        print "Bearing to target:", bearing
        
        self.out_ang = degrees(self.angularControl(bearing))
        self.out_lin = self.speedControl()
        pass

    def publish(self):
        print "Publishing!"
        print "Linear:", self.out_lin, 'realCommand:', abs(self.out_lin + 300) + 1000
        print "Angular:", self.out_ang, 'realCommand:', self.out_ang + 95
        

        vel_msg = Twist()
        vel_msg.angular.z = self.out_ang
        vel_msg.linear.x = self.out_lin
        
        self.vel_publisher.publish(vel_msg)

        pass
    
def init_current_node():
    rospy.init_node('robotcar', anonymous=True)

    robot = RobotCar()

    while not rospy.is_shutdown():
        robot.control()
        robot.publish()
        print "\n\n"
        rospy.sleep(0.1)

if __name__ == '__main__':
    init_current_node()