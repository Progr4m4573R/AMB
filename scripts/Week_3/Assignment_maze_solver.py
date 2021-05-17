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
        
        #Used to publis movement commands
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist,
                                           queue_size=1)

        #use to make the robot move
        self.twist = Twist()
    #Obstacle avoidance using lasers
    def laser_callback(self, incoming_data):
        # print len(incoming_data.ranges) CHECK COLOUR BEFORE TURNING
        if incoming_data.ranges[320] < 1.0:
            t = Twist()
            t.linear.x = 0
            t.angular.z = radians(90);#turn right at this speed
            self.p.publish(t)

        # #go forwards if there is nothing in front
        # elif incoming_data.ranges[320] > 1.0:
        #     t = Twist()
        #     t.linear.x = 0.5
        #     self.p.publish(t) 
        #     #calling goal searcher code
        #     self.image_callback
        # elif incoming_data.ranges[320] > 1.0: #function that returns mask colours
        #     t.linear.x = 0  
    
    #Looking for the goal 
    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')#open cv image that can be used by any open cv function
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)#converts that the robot sees to hsv

        lower_green = numpy.array([50, 100, 125])#detect green #hue, saturation and value,value has to be lower than 100 or image will be black
        upper_green = numpy.array([255, 255, 255])#this too

        lower_red = numpy.array([100, 100, 100])#detect red
        upper_red = numpy.array([255, 255, 255])#this too

        lower_blue = numpy.array([0, 200, 100])# detect blue
        upper_blue = numpy.array([255, 255, 255])#this too

        #Create a threshold for detecting the colours in a certain range, compare to hsv for deciding what to do when something is detected.
        bgr_thresh = cv2.inRange(image,
                            numpy.array((200, 230, 230)),#lower
                            numpy.array((255, 255, 255)))#upper
        #look for a hsv value between the ranges--------------------------------------------------------------
        #Set a threshold for detecting red
        hsv_red_thresh = cv2.inRange(hsv,lower_red,upper_red)
        #Set a threshold for detecting green
        hsv_green_thresh = cv2.inRange(hsv,lower_green,upper_green)
        #Set a threshold for detecting blue
        hsv_blue_thresh = cv2.inRange(hsv,lower_blue,upper_blue)
        
        # Instead find the contours in the mask generated from the
        # HSV image.
        _, hsv_contours, hierachy = cv2.findContours(
            hsv_blue_thresh.copy(),#find contours in red squares
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_red_thresh.copy(),#find contours in red squares
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        _, hsv_contours, hierachy = cv2.findContours(
            hsv_green_thresh.copy(),#find contours in red squares
            cv2.RETR_TREE,
            cv2.CHAIN_APPROX_SIMPLE)

        # in hsv_contours we now have an array of individual
        # closed contours (basically a polgon around the 
        # blobs in the mask). Let's iterate over all those found 
        # contours.
        for c in hsv_contours:
            # This allows to compute the area (in pixels) of a contour
            a = cv2.contourArea(c)
            # and if the area is big enough, we draw the outline
            # of the contour (in blue)
            if a > 100.0:
                cv2.drawContours(image, c, -1, (255, 0, 0), 3)
        #print('====')
        lower_yellow = numpy.array([10, 60, 70])
        upper_yellow = numpy.array([255, 255, 255])
        #Focus in on middle of line and move forwards while keeping a dot in the center of the line.
        mask = cv2.inRange(hsv,lower_green,upper_green)
        h, w, d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)  
       
        #This never runs because Mask i always less than 0
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 30
            print self.twist.angular.z
            print("moving... via moments")
            self.cmd_vel_pub.publish(self.twist)    
        else:#M is not greater than o  
            self.twist.linear.x = 0.5
            self.cmd_vel_pub.publish(self.twist)
            print("moving... w/o moments")


        
        cv2.imshow("window", image)
        cv2.waitKey(3)


rospy.init_node('receiver')
rec = Receiver()

rospy.spin()
cv2.destroyAllWindows()