#! /usr/bin/env python

"""
Programmer: Kim Yeonghoon
Refactoring follower_v1

Leader: raspi_0
Follower: raspi_1, raspi_2, and so on
"""

import rospy
import os
import math
import statistics
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FOLLOWER_NODE = 1       # Num of followers
CENTER_POS = 180        # Front. Never change.s
DETECTION_ANGLE = 30    # The angle of detection. Total range will be double of this
DISTANCE = 0.4          # Target distance
MARGIN = 0.05           # DISTANCE +- MARGIN will be the detection range
SPEED = 0.1             # Basic movement speed

class Follower:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_0/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.vel_pub = []
        for i in range(FOLLOWER_NODE):
            pub = rospy.Publisher('/raspi_'+ str(i + 1) +'/cmd_vel', Twist, queue_size=1)
            self.vel_pub.append(pub)
            

    def scan_callback(self, data):
        self.scan_data = data

    def object_detection(self):
        predict = 180

    # Write twist.angular value to adjust heading of the robot
    def adjust_heading(self, list):
        heading = CENTER_POS                # Set the default target heading as CENTER_POS
        if len(list) != 0:
            heading = 0
            for i in range(len(list)):
                heading += list[i]
            heading /= len(list)

        self.vel_msg.angular.z = math.radians((heading - CENTER_POS) * 2)

        rospy.loginfo(heading)
        

    # Write twist.linear value to adjust distance from the robot to the object to track
    def adjust_distance(self, angle_list, range_list):
        self.vel_msg.linear.x = SPEED

        for i in range(len(angle_list)):
            if range_list[angle_list[i]] < DISTANCE - MARGIN:
                self.vel_msg.linear.x = 0
                break

    def update(self):
        self.vel_msg.linear.x = 0.1
        angles_in_range = []                   # Contains the angle detected something within DISTANCE +- MARGIN

        for i in range(CENTER_POS - DETECTION_ANGLE, CENTER_POS + DETECTION_ANGLE + 1):
            if self.scan_data.ranges[i] > DISTANCE - MARGIN and self.scan_data.ranges[i] < DISTANCE + MARGIN:
                angles_in_range.append(i)      # Save the angle
            elif self.scan_data.ranges[i] < DISTANCE - MARGIN and self.scan_data.ranges[i] != 0:
                self.vel_msg.linear.x = 0

        self.adjust_heading(angles_in_range)
        self.vel_pub.publish(self.vel_msg)



if __name__ == '__main__':
    rospy.init_node('follower')
    rospy.loginfo('Follower node initialized.')

    follower = Follower()

    r = rospy.Rate(2)				# 2 Hz
    while not rospy.is_shutdown():
        follower.update()
        r.sleep()
