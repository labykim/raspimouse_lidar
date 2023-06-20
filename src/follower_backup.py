#! /usr/bin/env python

import rospy
import os
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

CENTER_POS = 180        # Front. Never change.
DETECTION_RANGE = 30    # The angle of detection. Total range will be double of this
DISTANCE = 0.4          # Target distance
MARGIN = 0.05           # 

class Follower:
    def __init__(self):
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_0/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()
        # self.is_detected = False

    def scan_callback(self, data):
        in_range = []           # Contains the angle detected something within DISTANCE +- MARGIN

        self.vel_msg.linear.x = 0.1

        for i in range(CENTER_POS - DETECTION_RANGE, CENTER_POS + DETECTION_RANGE + 1):
            if data.ranges[i] > DISTANCE - MARGIN and data.ranges[i] < DISTANCE + MARGIN:
                in_range.append(i)      # Save the angle
            elif data.ranges[i] < DISTANCE - MARGIN and data.ranges[i] != 0:
                self.vel_msg.linear.x = 0
        
        # rospy.loginfo(in_range)

        self.adjust_heading(in_range)
        # self.adjust_distance(in_range, data.ranges)

        self.vel_pub.publish(self.vel_msg)

    # Write twist.angular value to adjust heading of the robot
    def adjust_heading(self, list):
        heading = CENTER_POS
        if len(list) != 0:
            heading = 0
            for i in range(len(list)):
                heading += list[i]
            heading /= len(list)

        self.vel_msg.angular.z = math.radians((heading - CENTER_POS) * 2)

        rospy.loginfo(heading)
        

    # Write twist.linear value to adjust distance from the robot to the object to track
    def adjust_distance(self, angle_list, range_list):
        self.vel_msg.linear.x = 0.1

        for i in range(len(angle_list)):
            rospy.loginfo(angle_list[i])
            rospy.loginfo(range_list[angle_list[i]])
            if range_list[angle_list[i]] < DISTANCE - MARGIN:
                self.vel_msg.linear.x = 0
                break
            


def main():
    rospy.init_node('follower')
    rospy.loginfo('Follower node initialized.')

    follower = Follower()

    try:
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Update function
            r.sleep()

    except rospy.ROSInterruptException:
        pass    

if __name__ == '__main__':
    main()
    