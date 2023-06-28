#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

BOID_NUM = 2
SEPARATION_RANGE = 0.3

class Boid:
    def __init__(self, index):
        self.var_scan_callback = self.scan_callback_init
        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.var_scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        # Store the [0]angle and [1]range information of other boids
        # boid_pos[0] is always the leader
        self.boid_pos = [[] for _ in range(BOID_NUM)]

        rospy.loginfo("Subscriber & publisher for /raspi_" + str(index) + " established.")

    # scan_callback function only for the first time
    # Include initialization
    def scan_callback_init(self, data):
        self.data = data.ranges
        rospy.loginfo(self.data[175:185])

        self.init_boid_detection()

    def scan_callback(self, data):
        self.data_old = self.data
        self.data = data.ranges

        self.object_detection()

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


    # Detect a leader and other boids
    def object_detection(self):
        pass

    # Head to the same direction with the boids around
    def alignment(self):
        vector = []
        return vector

    # Gather with the boids around
    def cohesion(self):
        vector = []
        return vector

    # Keep distance with the boids around
    def separation(self):
        vector = []
        for i in range(len(self.boid_pos)):
            if self.boid_pos[i][1] < SEPARATION_RANGE:
                pass

        # Required data: direction vector from self to other boid

        return vector

    def update(self):
        vector = [0, 0]                         # Final cartesian vector (x, y)

        vector += self.alignment()
        vector += self.cohesion()
        vector += self.separation()

        # TODO: Convert the vector into Twist
        # Variable 'vector' contains the final Cartesian vector
        # Convert it into a polar vector and again into a Twist
        # Magnitude may directly be Twist.linear.x (in our case, self.vel_msg.linear.x)
        # Angle may directly be Twist.angular.z (in our case, self.vel_msg.angular.z)
        
        self.vel_pub.publish(self.vel_msg)
        self.vel_msg_old = self.vel_msg

    def condition_check(self):
        if self.boid_pos[0] == []:
            return False



if __name__ == '__main__':
    rospy.init_node('boids_master')
    rospy.loginfo('Boids master node initialized.')

    # Instantiate [BOID_NUM] Boid classes. Each class will be in charge of one raspimouse.
    # LEADER IS NOT IN THIS ARRAY
    boids = [Boid(i) for i in range(BOID_NUM)]

    # Loop for condition check before starting Boids
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        isReady = 1
        for i in range(BOID_NUM):
            if boids[i].condition_check(i) == False:
                rospy.loginfo("Leader is not detected. Place the leader boid in front of raspi_" + str(i))
                isReady = 0
                break
        if isReady:
            break
        r.sleep()

    rospy.loginfo("Initial condition satisfied. Starting Boids...")

    # Loop for live update
    # Nothing yet since the main algorithm is run by scan_callback
    # TODO: Maybe some superviser thing?
    r = rospy.Rate(5)
    while not rospy.is_shutdown():
        # something
        r.sleep()
