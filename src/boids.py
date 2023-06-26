#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

BOID_NUM = 2

class Boid:
    def __init__(self, index):
        self.var_scan_callback = self.scan_callback_init
        rospy.Subscriber('/raspi_' + str(index) + '/scan', LaserScan, self.var_scan_callback)
        self.vel_pub = rospy.Publisher('/raspi_' + str(index) + '/cmd_vel', Twist, queue_size=1)
        self.vel_msg = Twist()

        rospy.loginfo("Subscriber & publisher for /raspi_" + str(index) + "established.")

    # scan_callback function for the first time
    # Include initialization
    def scan_callback_init(self, data):
        self.data_new = data.ranges

        # Store the angle and range information of other boids
        # boid_pos[0] is always the leader
        self.boid_pos = [[] for _ in range(BOID_NUM)]

    def scan_callback(self, data):
        self.data_old = self.data_new
        self.data_new = data.ranges

        self.object_detection()

    def object_detection(self):
        pass

    # Head to the same direction with the boids around
    def alignment(self):
        vector = Twist()
        pass

    # Gather with the boids around
    def cohesion(self):
        pass

    # Keep distance with the boids around
    def separation(self):
        pass

    def update(self):
        
        self.vel_pub.publish(self.vel_msg)

if __name__ == '__main__':
    rospy.init_node('boids_master')
    rospy.loginfo('Boids master node initialized.')

    # Instantiate [BOID_NUM] Boid classes. Each class will be in charge of one raspimouse.
    boids = [Boid(i) for i in range(BOID_NUM)]

    r = rospy.Rate(1)
    isReady = -1
    while not rospy.is_shutdown():
        if isReady == 1:
            r.sleep()
            continue

        for i in range(1, BOID_NUM):
            isReady = 1
            if boids[i].boid_pos[0] == None:
                rospy.loginfo("Leader is not detected. Place the leader boid in front of raspi_" + str(i))
                isReady = 0
                break
