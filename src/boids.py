#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import coord

BOID_NUM = 2                    # Total number of robots
SEPARATION_RANGE = 0.4
NEIGHBOUR_RADIUS = 0.65

ALIGNMENT_WEIGHT = 1.0
COHESION_WEIGHT = 1.0
SEPARATION_WEIGHT = 1.0

VELOCITY_MULTIPLIER = 1.0

class Boid:
    """
    Each instance of this class will be in charge of every calculation and ordering for single boid (robot).
    """
    def __init__(self, index):
        self.isUpdating = True

        self.boid_pos = [[] for _ in range(BOID_NUM - 1)]               # [rho, phi]
        self.boid_pos_prev = [[] for _ in range(BOID_NUM - 1)]
        self.boid_vel = [[0, 0] for _ in range(BOID_NUM - 1)]
        # self.boid_vel_prev = [[] for _ in range(BOID_NUM - 1)]
        self.boid_vel_prev = self.boid_vel

        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.vel_cart = [0, 0]
        self.vel_pol = [0, 0]

        self.test_condition_init()

        rospy.loginfo("Instantiated: /raspi_" + str(index))
        self.isUpdating = False

    def scan_callback(self, data):
        """
        Callback function for `/scan`.
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
        self.boid_pos_prev = self.boid_pos
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
    
    def boid_vel_update(self):
        """
        Update the velocity of nearby boids.
        """
        self.boid_vel_prev = self.boid_vel

        for i in range(BOID_NUM-1):
            self.boid_vel[i] = coord.polar_velocity(self.boid_pos_prev[i], self.boid_pos[i], self.vel_pol)

    def alignment(self, vel: list) -> list:
        """
        Steer to same direction with nearby boids.

        Parameters
            `vel` A list of the nearby boids' current velocity

        Returns
            A cartesian vector [x, y]
        """
        vector = [0, 0]
        for i in range(len(self.boid_vel)):
            vector[0] = vector[0] + self.boid_vel[i][0]
            vector[1] = vector[1] + self.boid_vel[i][1]
        vector[0] = vector[0] / len(self.boid_vel)
        vector[1] = vector[1] / len(self.boid_vel)

        rospy.loginfo('Alignment : ' + str(vector))
        return vector

    def cohesion(self):
        """
        Gather with nearby boids; head to the center point.

        Returns
            A cartesian vector [x, y]
        """
        vector = [0, 0]
        
        count = 1
        for i in range(len(self.boid_pos)):
            if self.boid_pos[i][0] < NEIGHBOUR_RADIUS:
                count += 1
                tmp_vec = coord.pol_to_cart(self.boid_pos[i])
                vector[0] += tmp_vec[0]
                vector[1] += tmp_vec[1]
        
        if count != 1:
            vector[0] /= count
            vector[1] /= count

        rospy.loginfo('Cohesion  : ' + str(np.round(vector, 4)))
        return vector

    def separation(self):
        """
        Keep certain distance from nearby boids. 

        ### Parameters

        ### Returns
            A cartesian vector [x, y]
        """
        isExist = False
        fcv = [0, 0]                        # Final Cartesian Vector
        for i in range(len(self.boid_pos)):
            if self.boid_pos[i][0] < SEPARATION_RANGE:
                isExist = True
                v = coord.pol_to_cart([(SEPARATION_RANGE - self.boid_pos[i][0]) * SEPARATION_WEIGHT, self.boid_pos[i][1] + 180])
                fcv[0] = fcv[0] + v[0]
                fcv[1] = fcv[1] + v[1]
        
        if isExist:
            rospy.loginfo('Separation: ' + str(coord.cart_to_pol(fcv)))
        
        return fcv

    def vector_calculation(self):
        """
        Calulate the velocity for next movement of myself

        Returns
            (float) Polar vector (rho, phi)
        """
        self.vel_cart = [0, 0]
        # output = self.alignment(self.boid_vel)
        # self.vel_cart[0] += output[0]
        # self.vel_cart[1] += output[1]
        # output = self.cohesion()
        # self.vel_cart[0] += output[0]
        # self.vel_cart[1] += output[1]
        output = self.separation()
        self.vel_cart[0] += output[0]
        self.vel_cart[1] += output[1]

        # rospy.loginfo('Final C.V.: [' + str(np.round(self.vel_cart[0], 5)) + ', ' + str(np.round(self.vel_cart[1], 5)) + ']')

        self.vel_pol = coord.cart_to_pol(self.vel_cart)

        if self.vel_pol[1] < 0:
            self.vel_pol[1] += 360

        # rospy.loginfo('Final P.V.: ' + str(np.round(self.vel_pol[0], 5)) + ', ' + str(np.round(self.vel_pol[1], 5)) + ']')

        return self.vel_pol
    
    def publish(self, vector: list):
        """
        Write and publish a Twist topic with some post-processing:
        - Make it move backward, not rotating 180 degree
        - `VELOCITY_MULTIPLIER`

        ### Parameters
        vector Polar vector (rho, phi_d) that represents final velocity for next interval
        """
        if vector[1] > 90 and vector[1] < 270:
            vector[0] = -vector[0]
            vector[1] -= 180
        elif vector[1] >= 270 and vector[1] < 360:
            vector[1] -= 360

        

        # rospy.loginfo('FINAL: ' + str(vector))

        msg = Twist()
        msg.linear.x = vector[0] * VELOCITY_MULTIPLIER
        msg.angular.z = np.radians(vector[1])

        self.vel_pub.publish(msg)

    def update(self):
        """
        Analyze the current situation and calculate the next move.
        """
        if self.isUpdating:
            return
        self.isUpdating = True              # To block changes in boid_pos during calculation

        # Calculate the velocity of other boids
        self.boid_vel_update()

        rospy.loginfo(str(self.boid_pos))
        rospy.loginfo(str(self.boid_vel))

        vector = self.vector_calculation()

        self.publish(vector)

        # Update some attributes
        self.vel_msg_prev = self.vel_msg
        self.isUpdating = False
        
    def test_condition_init(self):
        """
        Allocate intial positions of nearby boids.
        Planned to be changed.
        """
        self.boid_pos[0] = [0.4, 0]           # Forward
        self.boid_pos_prev = self.boid_pos
        self.isUpdating = False

    def init_boid_detection(self):
        """
        Locate nearby boids only with the lightsensor data.
        """
        pass


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
