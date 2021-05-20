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
            t.angular.z = radians(45);#rotate right at this speed
            print("Turning left to avoid obstacle...")
            self.p.publish(t)
                            
        # #go forwards if there is nothing in front and priorities turning right
        elif incoming_data.ranges[320] > 1.0:
            
            t = Twist()
            t.linear.x = 0.5
            t.angular.z = radians(-25);#rotate right at this speed
            self.p.publish(t) 
            self.image_callback
            print("Exploring....")
             
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
        upper_blue = numpy.array([10, 255, 255])#this too

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

        #Focus in on middle of line and move forwards while keeping a dot in the center of the line.
        
        green_mask = hsv_green_thresh
        red_mask = hsv_red_thresh
        blue_mask = hsv_blue_thresh
        h, w, d = image.shape

        #create moments of all possible detections so i can check when one is detected
        gM = cv2.moments(green_mask)  
        rM = cv2.moments(red_mask)
        bM = cv2.moments(blue_mask)
        #This never runs because Mask i always less than 0
        if gM['m00'] > 0:
            print("goal detected!")
            cx = int(gM['m10']/gM['m00'])
            cy = int(gM['m01']/gM['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

            err = cx - w/2
            self.twist.linear.x = 0.5
            self.twist.angular.z = -float(err) / 100
            print self.twist.angular.z
            print("moving... via moments")
            self.cmd_vel_pub.publish(self.twist) 

        elif bM['m00'] > 0:
            print("RED DETECTED")
            print("EVASIVE MANEUVERS!!!!")
            self.twist.angular.x = 0.5
            self.twist.angular.z = radians(180)
        
        elif rM['m00'] > 0:
            print("Land mark detected")

        else:#M is not greater than 0 so the robot moves aimlessly without direction.  
            #"moving... w/o moments")
            return True

    def robot_control():
        #call image callback
        self.image_callback(self,msg)
        if self.image_callback() == True:
            self.laser_callback(self,self.incoming_data.ranges[320])
        
        cv2.imshow("window", image)
        cv2.waitKey(3)



rospy.init_node('receiver')
rec = Receiver()

rospy.spin()
cv2.destroyAllWindows()