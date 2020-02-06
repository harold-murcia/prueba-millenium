#!/usr/bin/env python

import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

origin = [0, 0]

def chargingMap(data):
    global origin
    print '- Charging Map'
    origin = [data.info.width/2, data.info.height/2]

def calculateOrigin(data):
    global origin
    print '- Calculate origin'
    origin = [data.pose.pose.position.x + origin[0], data.pose.pose.position.y + origin[1]]

def seeOdometry(data):
    print '- Robot position'
    print [data.pose.pose.position.x + origin[0], data.pose.pose.position.y + origin[1]]
    
def robotPosition():
    rospy.init_node('robot_position', anonymous=True)
    rospy.Subscriber('/roombot/map', OccupancyGrid, chargingMap)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, calculateOrigin)
    rospy.Subscriber('/t265/odom/sample', Odometry, seeOdometry)
    rospy.spin()

if __name__ == '__main__':
    robotPosition()

