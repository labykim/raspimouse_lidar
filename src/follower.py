#! /usr/bin/env python

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FOLLOWER_NUM = 2

class Follower():
    def __init__(self, index):
        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

    def scan_callback(self, data):
        self.scan_data = data

    def object_detection(self):
        pass

    def object_classification(self):
        pass

    def collision_prevention(self):
        pass

    def update(self):
        pass

if __name__ == '__main__':
    rospy.init_node('follower_master')
    rospy.loginfo('Follower Master node initialized.')

    follower = [Follower(i) for i in range(FOLLOWER_NUM)]

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # follower_essential.update()
        r.sleep()
