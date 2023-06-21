#! /usr/bin/env python

"""
Implement essential components only for multiple robot operation
This will be run on remote PC.
<Leader> /raspi_0/
    scan

<Follower> /raspi_1/
    scan
    cmd_vel
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

FOLLOWER_NODE = 2
CENTER_POS = 180        # Front. Never change.s
DETECTION_ANGLE = 30    # The angle of detection. Total range will be double of this
DISTANCE = 0.4          # Target distance
MARGIN = 0.05           # DISTANCE +- MARGIN will be the detection range
SPEED = 0.1             # Basic movement speed

class FollowerEssential():
    def __init__(self):
        self.vel_pub = []
        self.scan_data = [[] for i in range[FOLLOWER_NODE]]         # TODO: Is it correct?
        self.vel_msg = [Twist() for i in range[FOLLOWER_NODE]]      # TODO: Is it correct?
        for i in range(FOLLOWER_NODE):
            rospy.Subscriber('/raspi_' + str(i) + '/scan', LaserScan, self.scan_callback, i)
            self.vel_pub.append(rospy.Publisher('/raspi_'+ str(i) +'/cmd_vel', Twist, queue_size=1))
            rospy.loginfo('Subscriber and publisher for /raspi_' + str(i) + ' established.')

    def scan_callback(self, data, index):
        rospy.loginfo('scan_callback function from /raspi_' + str(index) + '/scan')
        self.scan_data[index] = data
        self.update(index, data)

    def leader_detection(self, index):
        # Catch the detection only in specific range of angle and distance
        angles_in_range = []                   # Contains the angle detected something within DISTANCE +- MARGIN

        for i in range(CENTER_POS - DETECTION_ANGLE, CENTER_POS + DETECTION_ANGLE + 1):
            if self.scan_data.ranges[i] > DISTANCE - MARGIN and self.scan_data.ranges[i] < DISTANCE + MARGIN:
                angles_in_range.append(i)      # Save the angle
            elif self.scan_data.ranges[i] < DISTANCE - MARGIN and self.scan_data.ranges[i] != 0:
                self.vel_msg.linear.x = 0
        
    def collision_prevention(self, index):
        # Use all directions
        # Never go to the angle which has detected the close contact
        pass

    def update(self, index):
        rospy.loginfo('Update function called by raspi_' + str(index) + '.')
        vel_msg = self.leader_detection(index)




if __name__ == '__main__':
    rospy.init_node('follower')
    rospy.loginfo('FollowerEssential node initialized.')

    follower_essential = FollowerEssential()

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # follower_essential.update()
        r.sleep()
