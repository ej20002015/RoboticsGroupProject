#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Vision:

    def __init__(self, display=False):

        self.colors = {} # colors we want to detect
        self.detected = {} # what we've detected

        self.display = display
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.update)

    def update(self, image):

        rgb_image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

        for color in self.colors.keys():

            bounds = self.colors[color]
            mask = cv2.inRange(hsv_image, bounds['lowerBound'], bounds['upperBound'])
            contours = cv2.findContours(mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)[0]

            if len(contours) > 0:

                max_contour = max(contours, key=cv2.contourArea)
                self.detected[color]['detected'] = True
                self.detected[color]['contourArea'] = cv2.contourArea(max_contour)

            else:

                self.detected[color]['detected'] = False
                self.detected[color]['contourArea'] = 0.0

        if self.display:

            cv2.namedWindow('rgb')
            cv2.imshow('rgb', rgb_image)
            cv2.waitKey(3)

    def setColorRanges(self, colorRanges):

        self.colors = colorRanges
        self.detected = {color:{'detected': False, 'contourArea': 0.0} for color in colorRanges}

    def detectColors(self):

        return self.detected