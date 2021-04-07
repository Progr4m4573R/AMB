# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 15:40:31 2015

@author: lcas
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import radians

class Receiver:

    def __init__(self):
        rospy.init_node('receiver')
        self.subscriber = rospy.Subscriber(
            '/scan', LaserScan, callback=self.cb)
        self.p = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=1)

    def cb(self, incoming_data):
        # print len(incoming_data.ranges)
        if incoming_data.ranges[320] < 1.0:
            t = Twist()
            t.linear.x = 0
            t.angular.z = radians(90);#turn right at this speed
            self.p.publish(t)


        #go forwards
        elif incoming_data.ranges[320] > 1.0:
            t = Twist()
            t.linear.x = 0.3
            self.p.publish(t)   

rec = Receiver()

rospy.spin()
