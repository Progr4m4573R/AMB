#!/usr/bin/env python

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image,
                                          self.image_callback)

    def image_callback(self, msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([10, 60, 70])
        upper_yellow = numpy.array([255, 255, 255])
        lower_blue = numpy.array([0, 200, 100])# detect blue
        upper_blue = numpy.array([10, 255, 255])#this too
        lower_red = numpy.array([100, 100, 100])#detect red
        upper_red = numpy.array([255, 255, 255])#this too
        lower_green = numpy.array([50, 100, 125])#detect green #hue, saturation and value,value has to be lower than 100 or image will be black
        upper_green = numpy.array([255, 255, 255])#this too
        mask = cv2.inRange(hsv, lower_red, upper_red)
        cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("window", mask)
        cv2.waitKey(3)

#cv2.startWindowThread()
rospy.init_node('follower')
follower = Follower()
rospy.spin()
cv2.destroyAllWindows()
