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
    Each instance of this class will be in charge of every calculation and ordering for each boid (robot).
    """
    def __init__(self, index):
        self.isUpdating = True

        self.boid_pos = [[] for _ in range(BOID_NUM - 1)]
        """ [rho, phi(deg)] """
        self.boid_vel = [[0, 0] for _ in range(BOID_NUM - 1)]
        """ [x, y] """
        self.boid_pos_prev = self.boid_pos
        self.boid_vel_prev = self.boid_vel

        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)

        self.vel_cart = [0, 0]
        self.vel_pol = [0, 0]

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
            `vel` List of the nearby boids' velocity (i.e., Cartesian vector) for last interval.

        Returns
            Cartesian vector `[x, y]`
        """
        vector = [0, 0]
        for i in range(len(vel)):
            vector[0] = vector[0] + vel[i][0]
            vector[1] = vector[1] + vel[i][1]
        vector[0] = vector[0] / len(vel)
        vector[1] = vector[1] / len(vel)

        rospy.loginfo('Alignment : ' + str(vector))
        return vector

    def cohesion(self, pos: list) -> list:
        """
        Gather with nearby boids; head to the center point.

        Parameters
            `pos` Position of nearby boids; [distance, angle in degrees]

        Returns
            A cartesian vector `[x, y]`
        """
        vector = [0, 0]
        
        count = 1
        for i in range(pos):
            if pos[i][0] < NEIGHBOUR_RADIUS:
                count += 1
                tmp_vec = coord.pol_to_cart(pos[i])
                vector[0] += tmp_vec[0]
                vector[1] += tmp_vec[1]
        
        if count != 1:
            vector[0] /= count
            vector[1] /= count

        rospy.loginfo('Cohesion  : ' + str(np.round(vector, 4)))
        return vector

    def separation(self, pos: list) -> list:
        """
        Keep certain distance from nearby boids. 

        ### Parameters
            `pos` Position of nearby boids; [distance, angle in degrees]

        ### Returns
            A cartesian vector `[x, y]`
        """
        isExist = False
        fcv = [0, 0]                        # Final Cartesian Vector
        for i in range(len(pos)):
            if pos[i][0] < SEPARATION_RANGE:
                isExist = True
                v = coord.pol_to_cart([(SEPARATION_RANGE - pos[i][0]) * SEPARATION_WEIGHT, pos[i][1] + 180])
                fcv[0] = fcv[0] + v[0]
                fcv[1] = fcv[1] + v[1]
        
        if isExist:
            rospy.loginfo('Separation: ' + str(coord.cart_to_pol(fcv)))
        
        return fcv

    def vector_calculation(self):
        """
        Calulate the velocity for next movement.

        Returns
            Polar vector [rho, phi_deg]
        """
        cv = [0, 0]
        output = self.alignment(self.boid_vel)
        cv[0] += output[0]
        cv[1] += output[1]
        output = self.cohesion(self.boid_pos)
        cv[0] += output[0]
        cv[1] += output[1]
        output = self.separation(self.boid_pos)
        cv[0] += output[0]
        cv[1] += output[1]

        pv = coord.cart_to_pol(cv)

        # rospy.loginfo('Final C.V.: [' + str(np.round(cv[0], 5)) + ', ' + str(np.round(cv[1], 5)) + ']')
        # rospy.loginfo('Final P.V.: [' + str(np.round(pv[0], 5)) + ', ' + str(np.round(pv[1], 5)) + ']')

        return pv
    
    def publish(self, pv: list):
        """
        Write and publish a Twist topic with some post-processing:
        - Decide whether move forward or backward
        - Use `VELOCITY_MULTIPLIER`

        ### Parameters
            `pv` Polar vector [rho, phi_d] that represents velocity for next interval
        """
        if pv[1] > 90 and pv[1] < 270:
            pv[0] = -pv[0]
            pv[1] -= 180
        elif pv[1] >= 270 and pv[1] < 360:
            pv[1] -= 360

        pv[1] = np.radians(pv[1])

        rospy.loginfo('Publishing: ' + str(pv))

        msg = Twist()
        msg.linear.x = pv[0] * VELOCITY_MULTIPLIER
        msg.angular.z = pv[1]

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

        self.isUpdating = False
        
    def test_condition_init(self):
        """
        Allocate intial positions of nearby boids.
        Planned to be changed.
        """
        self.boid_pos[0] = [0.4, 0]           # Forward
        # self.boid_pos_prev = self.boid_pos
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
