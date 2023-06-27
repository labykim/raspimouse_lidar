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

        # Store the angle and range information of other boids
        # boid_pos[0] is always the leader
        self.boid_pos = [[] for _ in range(BOID_NUM)]

        rospy.loginfo("Subscriber & publisher for /raspi_" + str(index) + "established.")

    # scan_callback function for the first time
    # Include initialization
    def scan_callback_init(self, data):
        self.data_new = data.ranges

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

    def condition_check(self, index):
        if self.boid_pos[0] == None:
            rospy.loginfo("Leader is not detected. Place the leader boid in front of raspi_" + str(index))



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
