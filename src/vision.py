#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Vision:

    def __init__(self, **kwargs):

        self.displayRGB = kwargs.get('rgb', False)
        self.displayKP = kwargs.get('kp', False)
        self.displayMasks = kwargs.get('masks', False)

        self.colors = {} # colors we want to detect
        self.detector = cv2.ORB()
        self.matcher = cv2.BFMatcher()

        self.rgbImage = None
        self.initializeWindows()

        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.update)

    def initializeWindows(self):

        cv2.destroyAllWindows()
        cv2.namedWindow('rgb')
        cv2.namedWindow('kp')
        for color in self.colors.keys() : cv2.namedWindow('rgbMask-' + color)

    def update(self, image):

        self.rgbImage = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        if self.displayRGB:
            cv2.imshow('rgb', self.rgbImage)
            cv2.waitKey(3)

        if self.displayKP:
            keypoints = self.detector.detect(self.rgbImage)
            keypoints_img = cv2.drawKeypoints(self.rgbImage, keypoints, outImage=None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_OVER_OUTIMG)
            cv2.imshow('kp', keypoints_img)
            cv2.waitKey(3)

    def setColorRanges(self, colorRanges):

        self.colors = colorRanges
        self.initializeWindows()

    def detectColors(self):

        hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        detected = {color:{'detected': False, 'contourArea': 0.0} for color in self.colors}

        for color in self.colors.keys():

            bounds = self.colors[color]
            mask = cv2.inRange(hsvImage, bounds['lowerBound'], bounds['upperBound'])
            maskDisplay = cv2.bitwise_and(self.rgbImage, self.rgbImage, mask=mask)

            if self.display:
                cv2.imshow('rgbMask-' + color, maskDisplay)
                cv2.waitKey(3)

            contours = cv2.findContours(mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)[0]

            if len(contours) > 0:

                max_contour = max(contours, key=cv2.contourArea)
                detected[color]['detected'] = True
                detected[color]['contourArea'] = cv2.contourArea(max_contour)

            else:

                detected[color]['detected'] = False
                detected[color]['contourArea'] = 0.0
        
        return detected
    
    def detectCharacters(self):

        pass

    def onShutdown(self):

        cv2.destroyAllWindows()