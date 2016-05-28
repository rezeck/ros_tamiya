#!/usr/bin/env python
import rospy

from math import radians, cos, sin, asin, sqrt, atan2, degrees, pi
import pid_class

from ros_tamiya.msg import Gps_msg
from ros_tamiya.msg import Imu
from geometry_msgs.msg import Twist

robot = None

class RobotCar:
    def __init__(self):
        self.VMAX_PERCENT = 20 # max velocity percentage
        self.WAYPOINT_RADIUS = 2.0 # [m]
        
        # Gains linear vel
        self.KP_VEL = -5.0 
        self.TI_VEL = 80.0
        self.TD_VEL = 0*0.00005

        # Gains angular vel
        self.KP_PSI = -0.7
        self.TI_PSI = 70.0
        self.TD_PSI = 0*0.00007

        self.MAX_STEERING = 90.0 # max sterring angle

        self.MAGNETIC_DEFLECTION = 123

        # Lat, lon
        self.gps_target = [-19.868451, -43.966535]
        self.gps_pos = [-19.868828, -43.967068]
                        #-19.869544, -43.966333
                        #-19.868101, -43.966955
                        #-19.868828, -43.967068

        self.orientation = 0
        self.speed = 0
        self.phi = 0
        self.theta = 0
        self.psi = 0

        # Output values angular and linear
        self.out_ang = 0
        self.out_lin = 0

        self.pid_angle = pid_class.PID(self.KP_PSI, self.TI_PSI, self.TD_PSI, -self.MAX_STEERING, self.MAX_STEERING)
        self.pid_vel   = pid_class.PID(self.KP_VEL, self.TI_VEL, self.TD_VEL, 0.0, self.VMAX_PERCENT)
        
        self.vel_publisher = rospy.Publisher('vel_cmd', Twist, queue_size=1)
        pass

    def setGPS(self, data):
        #print 'setGPS data:', data.latitude, data.longitude 
        self.latitude = data.latitude
        self.longitude = data.longitude
        self.orientation = data.orientation
        self.speed = (0.514444*data.speed) # from knots to meters/sec

    def setIMU(self, data):
        #print 'setIMU data:', data.heading
        self.phi = data.gyroscope
        self.theta = data.accelerometer
        
        self.psi = (atan2(data.magnetometer.y, data.magnetometer.x) * 180) / pi
        if self.psi < 0:
            self.psi += 360
        
        #self.psi = data.heading
        
        #self.psi = self.psi + self.MAGNETIC_DEFLECTION
        #self.psi = self.psi % (2.0 * math.pi)

    def set_linear_vel(self, vel):
        self.lin_vel = vel
        pass

    def set_angular_vel(self, vel):
        self.ang_vel = vel
        pass

    def angularControl(self, psi_ref):
        alpha = psi_ref - self.psi

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
            while(True):
                print "REACHED!!!, get new WP"
        
        return self.pid_vel.u(rho)

    def calcBearingToTarget(self):
        """
        Calculate the angle between two points 
        on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [self.gps_pos[1], self.gps_pos[0], self.gps_target[1], self.gps_target[1]])

        dLon = lon2 - lon1
        y = sin(dLon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)
        return atan2(y, x)

    def haversineDistanceToTarget(self):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        """
        # convert decimal degrees to radians 
        lon1, lat1, lon2, lat2 = map(radians, [self.gps_pos[1], self.gps_pos[0], self.gps_target[1], self.gps_target[1]])

        # haversine formula 
        dlon = lon2 - lon1 
        dlat = lat2 - lat1 
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * asin(sqrt(a)) 
        r = 6371 # Radius of earth in kilometers. Use 3956 for miles
        return (c * r) * 1000

    def control(self):
        print "Compass:", self.psi

        bearing = degrees(self.calcBearingToTarget())
        if bearing < 0:
            bearing += 360
        print "Bearing:", bearing

        psiref = self.psi - bearing
        print "Dif:", psiref
        
        self.out_ang = degrees(self.angularControl(psiref)) #[-90..90]
        self.out_lin = self.speedControl()
        pass

    def publish(self):
        print "Publishing!"
        print "Angular:", self.out_ang
        print "Linear:", self.out_lin
        print "\n\n"

        vel_msg = Twist()
        vel_msg.angular.z = self.out_ang
        vel_msg.linear.x = self.out_lin
        self.vel_publisher.publish(vel_msg)

        pass
    
def init_current_node():
    rospy.init_node('robotcar', anonymous=True)

    robot = RobotCar()
    rospy.Subscriber("gps", Gps_msg, robot.setGPS)
    rospy.Subscriber("imu", Imu, robot.setIMU)

    while not rospy.is_shutdown():
        robot.control()
        robot.publish()
        rospy.sleep(1.0)

if __name__ == '__main__':
    init_current_node()