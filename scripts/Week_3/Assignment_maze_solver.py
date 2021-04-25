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
            #calling goal searcher code
            self.image_callback

    #Looking for the goal 
    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#open cv image that can be used by any open cv function
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)#converts that the robot sees to hsv

        lower_green = numpy.array([50, 100, 50])#detect green #hue, saturation and value,value has to be lower than 100 or image will be black
        upper_green = numpy.array([255, 255, 255])#this too

        lower_red = numpy.array([0, 100, 50])#detect red
        upper_red = numpy.array([255, 255, 255])#this too

        lower_blue = numpy.array([230, 100, 50])# detect blue
        upper_blue = numpy.array([255, 255, 255])#this too


        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)#look for a hsv value between the ranges...
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

    #if robot sees red do this, if green do this , if blue do this
    #def colour_check(lower, upper):
        #red
        #if lower is [170, 100, 100] and upper is [10, 255, 250]:
        #blue
        #if lower is [110, 50, 50] and upper is [130, 255, 255]:
        #green
        #if lower is [50, 100, 100] and upper is [70, 255, 255]:

rospy.init_node('receiver')
rec = Receiver()

rospy.spin()
