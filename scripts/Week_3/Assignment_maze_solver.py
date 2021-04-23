# -*- coding: utf-8 -*-
"""
Created on Wed Jan 28 15:40:31 2015

@author: lcas
"""
import numpy
import cv2
import cv_bridge
import rospy
from sensor_msgs.msg import Image


from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from math import radians

class Receiver:

    def __init__(self):
        
        #subscribers
        self.subscriber = rospy.Subscriber(
            '/scan', LaserScan, callback=self.laser_callback)

        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image,
                                          self.image_callback)

        #publishers
        self.p = rospy.Publisher(
            '/mobile_base/commands/velocity', Twist, queue_size=1)

        #Library Used to convert from ROS images to OpenCV
        self.bridge = cv_bridge.CvBridge()

    #Obstacle avoidance using lasers
    def laser_callback(self, incoming_data):
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
            #image_callback(self, msg)

    #Looking for the goal 
    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 10, 10])# change this to detect green
        upper_yellow = numpy.array([255, 255, 250])#this too
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 30
            print self.twist.angular.z

            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow("window", image)
        cv2.waitKey(3)

rospy.init_node('receiver')
rec = Receiver()

rospy.spin()
