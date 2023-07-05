#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import os
import coord

BOID_NUM = 2

SEPARATION_RANGE = 0.3
NEIGHBOUR_RADIUS = 0.6

ALIGNMENT_WEIGHT = 1.0
COHESION_WEIGHT = 1.0
SEPARATION_WEIGHT = 1.0

VELOCITY_MULTIPLIER = 1.0

class Boid:
    """
    Each instance of this class will be in charge of every calculation and ordering for one boid (robot).
    """
    def __init__(self, index):
        self.isUpdating = True
        self.var_scan_callback = self.init_scan_callback
        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.var_scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.boid_pos = [[] for _ in range(BOID_NUM - 1)]               # [rho, phi]
        self.boid_pos_prev = [[] for _ in range(BOID_NUM - 1)]
        self.boid_vel = [[] for _ in range(BOID_NUM - 1)]
        self.boid_vel_prev = [[] for _ in range(BOID_NUM - 1)]

        rospy.loginfo("Instantiated: /raspi_" + str(index))

    def init_scan_callback(self, data):
        """
        Callback function for /scan only for the first time.
        """
        self.data = data.ranges
        self.test_condition_init()
        self.tracking()

    def scan_callback(self, data):
        """
        Normal callback function for /scan.
        """
        if self.isUpdating:
            return
        self.data = data.ranges
        self.tracking()
        self.boid_vel_update()

    def tracking(self):
        """
        Process LaserScan data to calculate the position of nearby boids.
        """
        wide = 20

        for i in range(len(self.boid_pos_prev)):
            min_distance = 9
            hit_angle = 0
            prev_angle = int(self.boid_pos_prev[i][1])
            for j in range(prev_angle - wide, prev_angle + wide):
                if self.data[j] < min_distance and self.data[j] > 0:
                    min_distance = self.data[j]
                    hit_angle = j

            self.boid_pos[i] = [self.data[hit_angle], hit_angle]
        
        rospy.loginfo(str(self.boid_pos))
            
    def velocity_polar(self, p1, p2):
        """
        Calculate the velocity of from p1 to p2 in Polar coordinate.

        Args
            p1, p2 : Polar vector (rho, phi)
        Returns
            (float) Cartesian vector (x, y) from p1 to p2
        """
        c1 = coord.pol_to_cart(p1)
        c2 = coord.pol_to_cart(p2)
        return c2 + self.vel_cart * 0.2 - c1
    
    def boid_vel_update(self):
        """
        Update the velocity of tracking boids.
        """
        

    def alignment(self):
        """
        Steer to same direction with nearby boids.

        Returns
            A cartesian vector [x, y]
        """
        vector = [0, 0]


        # rospy.loginfo('Alignment vector: ' + str(vector))
        return vector

    def cohesion(self):
        """
        Gather with nearby boids; head to the center point.

        Returns
            A cartesian vector [x, y]
        """
        vector = [0, 0]

        # rospy.loginfo('Cohesion vector: ' + str(vector))
        return vector

    def separation(self):
        """
        Keep certain distance from nearby boids. 

        Returns
            A cartesian vector [x, y]
        """
        isExist = False
        vector = [0, 0]
        for i in range(len(self.boid_pos)):
            if self.boid_pos[i][0] < SEPARATION_RANGE:
                isExist = True
                # Recheck the direction of input polar vector
                vector += coord.pol_to_cart((self.boid_pos[i][0] - SEPARATION_RANGE) * SEPARATION_WEIGHT, self.boid_pos[i][1] - 180)
        
        if isExist:
            rospy.loginfo('Separation vector: ' + str(vector))
        
        return vector

    def vector_calculation(self):
        """
        Calulate the velocity for next movement of myself

        Returns
            (float) Polar vector (rho, phi)
        """
        self.vel_cart = [0, 0]
        output = self.alignment()
        self.vel_cart[0] += output[0]
        self.vel_cart[1] += output[1]
        output = self.cohesion()
        self.vel_cart[0] += output[0]
        self.vel_cart[1] += output[1]
        output = self.separation()
        self.vel_cart[0] += output[0]
        self.vel_cart[1] += output[1]

        vel_pol = coord.cart_to_pol(self.vel_cart[0], self.vel_cart[1])
        rospy.loginfo('Final polar vector: ' + str(vel_pol))
        return vel_pol

    def update(self):
        if self.isUpdating:
            return
        self.isUpdating = True

        # Calculate the velocity of other boids
        # ...

        vector = self.vector_calculation()

        # Convert the final polar vector to Twist
        self.vel_msg.linear.x = vector[0] * VELOCITY_MULTIPLIER
        self.vel_msg.angular.z = vector[1]
        
        # self.vel_pub.publish(self.vel_msg)          # Publish

        # Update some attributes
        self.vel_msg_prev = self.vel_msg
        self.boid_pos_prev = self.boid_pos
        self.isUpdating = False
        
    def test_condition_init(self):
        self.boid_pos[0] = [0.4, 0]           # Forward
        self.boid_pos_prev = self.boid_pos
        self.isUpdating = False



def main():
    rospy.init_node('boids_master')
    rospy.loginfo('Boids master node initialized.')

    boids = [Boid(i) for i in range(BOID_NUM)]

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        for i in range(BOID_NUM):
            boids[i].update()
        r.sleep()

if __name__ == '__main__':
    main()
