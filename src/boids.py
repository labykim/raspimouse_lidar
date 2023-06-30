#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import custom_np

BOID_NUM = 3

SEPARATION_RANGE = 0.3
NEIGHBOUR_RADIUS = 0.6

ALIGNMENT_WEIGHT = 1.0
COHESION_WEIGHT = 1.0
SEPARATION_WEIGHT = 1.0

VELOCITY_MULTIPLIER = 1.0

class Boid:
    def __init__(self, index):
        self.var_scan_callback = self.init_scan_callback
        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.var_scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        self.boid_pos = [[] for _ in range(BOID_NUM - 1)]       # [range, angle]
        self.boid_vel = [[] for _ in range(BOID_NUM - 1)]       # [pho, phi]
        self.boid_pos_prev = self.boid_pos
        self.boid_vel_prev = self.boid_vel

        rospy.loginfo("Instantiated: /raspi_" + str(index))

        self.test_condition_init()

    def init_scan_callback(self, data):
        self.data = data.ranges
        self.boid_detection()

    def scan_callback(self, data):
        if self.isUpdating:
            return
        self.data_old = self.data
        self.data = data.ranges

    def init_boid_detection(self):
        

        leader_pos = [180, 0.5]
        margin = [20, 0.05]
        hit = []
        
        for i in range(leader_pos[0] - margin[0], leader_pos[0] + margin[0]):
            if self.data[i] > leader_pos[1] - margin[1] and self.data[i] < leader_pos[1] + margin[1]:
                hit.append(i)

        mean_angle = sum(hit) / len(hit)
        range_sum = 0
        for i in range(hit):
            range_sum += self.data[hit[i]]
        mean_range = sum(range_sum) / len(range_sum)
        self.boid_pos[0] = [mean_angle,mean_range]

        for b in range(BOID_NUM - 1):
            pass

    def boid_detection(self):
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

    def boid_detection_discard(self):
        range_gap = 0.05
        hit = []

        start = 0
        for i in range(len(self.data)):
            if i == start:
                continue
            elif self.data[i] > 0.7:
                start = i + 1
            elif i > start and self.data[i] < self.data[i-1] + range_gap and self.data[i] > self.data[i-1] - range_gap:
                i += 1
            else:
                if i - start >= 4 and i - start <= 15:
                    hit.append([start, i])
                start = i + 1
        rospy.loginfo(str(hit))
        # rospy.loginfo(str(self.data[180]))

        for i in range(len(self.boid_pos_prev)):
            for j in range(len(hit)):
                if self.boid_pos_prev[i][0] > hit[j][0] and self.boid_pos_prev[i][0] < hit[j][1]:
                    midpoint = (hit[j][0] + hit[j][1]) / 2
                    midpoint = int(midpoint)
                    distance = self.data[midpoint]
                    self.boid_pos[i] = [distance, midpoint]

        # rospy.loginfo(str(self.boid_pos))
            

    # Calculate the velocity of other voids
    def boid_tracking(self):
        for i in range(len(self.boid_pos)):
            p1 = custom_np.pol_to_cart(self.boid_pos_prev[i][0], self.boid_pos_prev[i][1])
            p2 = custom_np.pol_to_cart(self.boid_pos[i][0], self.boid_pos[i][1])
            self.boid_vel[i] = p2 - p1
            rospy.logdebug('Velocity of boid[' + str(i) + ']: ' + str(self.boid_pos[i][2]))


    # Head to the same direction with the boids around
    def alignment(self):
        vector = [0, 0]
        rospy.logdebug('Alignment vector: ' + str(vector))
        return vector

    # Gather with the boids around
    def cohesion(self):
        vector = [0, 0]
        rospy.logdebug('Cohesion vector: ' + str(vector))
        return vector

    # Keep distance with the boids around
    def separation(self):
        vector = [0, 0]
        for i in range(len(self.boid_pos)):
            if self.boid_pos[i][0] < SEPARATION_RANGE:
                # NO NEED to inverse the angle since the lidar is attached reversely
                vector += custom_np.pol_to_cart(self.boid_pos[i][0], self.boid_pos[i][1]) * SEPARATION_WEIGHT
        
        rospy.logdebug('Separation vector: ' + str(vector))
        return vector

    def boid_vector(self):
        vector = [0, 0]                         # Final cartesian vector (x, y)

        vector += self.alignment()
        vector += self.cohesion()
        vector += self.separation()

        # (x, y) to (rho, phi)
        vector = custom_np.cart_to_pol(vector)
        rospy.logdebug('Final polar vector: ' + str(vector))
        return vector

    def update(self):
        self.isUpdating = True              # Block other processes

        self.test_condition_init()

        # Calculate the velocity of other boids
        # self.boid_tracking()

        # Calculate the velocity to move
        vector = self.boid_vector()

        # Convert the final polar vector to Twist
        self.vel_msg.linear.x = vector[0] * VELOCITY_MULTIPLIER
        self.vel_msg.angular.z = vector[1]
        
        # Publish
        # self.vel_pub.publish(self.vel_msg)

        # Before ending
        self.vel_msg_prev = self.vel_msg
        self.boid_pos_prev = self.boid_pos
        self.isUpdating = False

    def condition_check(self):
        if self.boid_pos[0] == []:
            return False
        
    def test_condition_init(self):
        self.boid_pos[0] = [0.4, 180]
        self.boid_pos[1] = [0.4, 270]
        self.boid_pos_prev = self.boid_pos



def main():
    rospy.init_node('boids_master')
    rospy.loginfo('Boids master node initialized.')

    boids = [Boid(i) for i in range(BOID_NUM)]

    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # for i in range(BOID_NUM):
        #     boids[i].update()
        r.sleep()

def test():
    rospy.loginfo('Starting code test...')
    
    rospy.init_node('boids_master')
    rospy.loginfo('Boids master node initialized.')

    boids = [Boid(i) for i in range(BOID_NUM)]

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        isReady = 1
        for i in range(BOID_NUM):
            if boids[i].condition_check(i) == False:
                
                isReady = 0
                break
        if isReady:
            break
        r.sleep()

    rospy.loginfo("Initial condition satisfied. Starting Boids...")

if __name__ == '__main__':
    main()
