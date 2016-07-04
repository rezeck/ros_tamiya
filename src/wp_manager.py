#!/usr/bin/python

import os
import math

class WPManager:
    def __init__(self, VERBOSE=0, wp_file='/tmp/wp_list.txt', is_plot_active=True, reached_radius=2):
        self.VERBOSE = VERBOSE

        self.is_completed = False
        self.managerStatus = 0
        self.distance = 0
        self.waypointListSize = 0

        self.waypointFile = wp_file
        self.plotActive = is_plot_active
        self.reachedRadius = reached_radius

        # Get the Waypoint list and externalWaypoint list from file
        self.getWaypointFromFile()

    # Calcute the distance (m) between Waypoint
    def dist(self, lat1, lon1, lat2, lon2):
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)

        lon2 = math.radians(lon2)
        x = (lon2-lon1) * math.cos((lat1+lat2)/2)
        y = (lat2-lat1)
        d = math.sqrt(x**2+y**2) * 6378137
        return d # in meters

    def printStatus(self):
        print "\33[1;91m                              Waypoint Status                                   \33[0m"
        print "\33[1;91m________________________________________________________________________________\33[0m"
        print "Current Waypoint Index             :", self.currentWaypointIndex, " of ", self.waypointListSize-1
        print "Distance calculated by manager     :", self.distance
        print "Current Waypoint                   :", self.currentWaypoint['lat'], self.currentWaypoint['lng']
        print "\33[1;91m________________________________________________________________________________\33[0m"

    # This function get the /ext and the /hum*(from respective quadrotor) waypoints from main waypoint file and save in two separated list
    def getWaypointFromFile(self):
        if self.VERBOSE:
            print "Loading Waypoints from ", self.waypointFile

        dataInputsFile = open(self.waypointFile, "r").read()
        dataInputsFile = dataInputsFile.split('\n')
        self.externalWaypointList = []
        self.waypointList = []

        for line in dataInputsFile:
            line = line.split(',')
            if len(line) == 1:
                continue
            name = line[0]
            lat = float(line[0])
            lng = float(line[1])
            waypoint = {}
            waypoint['lat'] = float(lat)
            waypoint['lng'] = float(lng)
            
            self.waypointList.append(waypoint)

        self.waypointListSize = len(self.waypointList)
        self.currentWaypointIndex = 0
        self.currentWaypoint = self.waypointList[self.currentWaypointIndex]

        # Sets the current quads waypointList_size_param
        print "A total of " + str(self.waypointListSize) + " external Waypoints was loaded.\n"

    def isWayPointReached(self, lat, lng):
        curr_wp = self.waypointList[self.currentWaypointIndex]
        self.distance = self.dist(curr_wp['lat'], curr_wp['lng'], lat, lng)

        status = False
        if(self.distance <= self.reachedRadius):
            status = True
            self.setNextWaypoint()

        return status

    def getDistanceToTarget(self, lat=None, lng=None):
        if lat is not None and lng is not None:
            self.distance = self.dist(curr_wp['lat'], curr_wp['lng'], lat, lng)
        
        return self.distance

    def isCompleted(self):
        return self.is_completed

    def getCurrentWayPoint(self):
	return [self.currentWaypoint['lat'], self.currentWaypoint['lng']]

    def setNextWaypoint(self):
        if self.currentWaypointIndex >= self.waypointListSize:
            print 'Error - waypoint index > len(list)'
            return False
        elif self.currentWaypointIndex == self.waypointListSize - 1:
            print 'Warning - last waypoint reached!'
            self.is_completed = True
            return False
        else:
            self.currentWaypointIndex += 1
            self.currentWaypoint = self.waypointList[self.currentWaypointIndex]
            return True


if __name__ == "__main__":

    # Intialize the waypoint manager
    # Testing
    wm = WPManager(VERBOSE=1)
    print 'WP REACHED?:', wm.isWayPointReached(-19.8597099,-43.9658604), 'WP ENDED?:', wm.isCompleted()
    wm.printStatus()
    print 'WP REACHED?:', wm.isWayPointReached(-19.8694744024, -43.9583882628), 'WP ENDED?:', wm.isCompleted()
    wm.printStatus()
    print 'WP REACHED?:', wm.isWayPointReached(-19.8694814982, -43.9583890558), 'WP ENDED?:', wm.isCompleted()
    wm.printStatus()
    