#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Vision:

    wallMaskSensitivity = 50
    wallMaskLowerBound = np.array([0, wallMaskSensitivity, 0])
    wallMaskUpperBound = np.array([255, 255, 255 - wallMaskSensitivity])

    def __init__(self, **kwargs):

        self.displayRGB = kwargs.get('rgb', False)
        self.displayMasks = kwargs.get('masks', False)

        # colors we want to detect
        self.colors = {}

        # Filepaths and identifiers of the images containing the objects we want to detect
        self.objects = {}

        # Object indentifiers mapped to upper and lower HSV mask bounds
        self.objectDetectionData = {}

        self.rgbImage = None
        self.initializeWindows()

        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.update)

    def initializeWindows(self):

        cv2.destroyAllWindows()
        cv2.namedWindow('rgb')

        if self.displayMasks:
            for color in self.colors.keys() : cv2.namedWindow('rgbMask-' + color)
            cv2.namedWindow("wallMask")
            cv2.namedWindow("objectMask")

    def update(self, image):

        self.rgbImage = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        if self.displayRGB:
            cv2.imshow('rgb', self.rgbImage)
            cv2.waitKey(3)

    def setColorRanges(self, colorRanges):

        self.colors = colorRanges
        self.initializeWindows()

    def setObjectRecognitionImagePaths(self, objectRecognitionImagePaths):

        self.objects = objectRecognitionImagePaths
        self.objectDetectionData.clear()
        for identifier, details in objectRecognitionImagePaths.items():
            img = cv2.imread(details["filepath"])

            self.objectDetectionData[identifier] = {
                "mask": (details["lowerBound"], details["upperBound"])
            }

    def detectColors(self):

        hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        detected = {color:{'detected': False, 'contourArea': 0.0} for color in self.colors}

        for color in self.colors.keys():

            bounds = self.colors[color]
            mask = cv2.inRange(hsvImage, bounds['lowerBound'], bounds['upperBound'])
            maskDisplay = cv2.bitwise_and(self.rgbImage, self.rgbImage, mask=mask)

            if self.displayMasks:
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
    
    def detectPresenceOfObjects(self):

        hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        hsvWallMask = cv2.inRange(hsvImage, Vision.wallMaskLowerBound, Vision.wallMaskUpperBound)
        wallMaskedImage = cv2.bitwise_and(self.rgbImage, self.rgbImage, mask=hsvWallMask)
        hsvWallMaskedImage = cv2.cvtColor(wallMaskedImage, cv2.COLOR_BGR2HSV)
        if self.displayMasks:
            cv2.imshow("wallMask", wallMaskedImage)

        detected = {obj:{'detected': False, 'contourArea': 0.0, "side": "NONE"} for obj in self.objectDetectionData}

        for identifier, details in self.objectDetectionData.items():

            objectMask = cv2.inRange(hsvWallMaskedImage, details['mask'][0], details['mask'][1])
            if self.displayMasks:
                objectMaskImage = cv2.bitwise_and(wallMaskedImage, wallMaskedImage, mask=objectMask)
                cv2.imshow("objectMask", objectMaskImage)

            contours = cv2.findContours(objectMask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)[0]
            if len(contours) > 0:
                maxContour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(maxContour) > 20:

                    moment = cv2.moments(maxContour)
                    contourX = int(moment["m10"]/moment["m00"])
                    centreX = self.rgbImage.shape[1] / 2

                    detected[identifier]["detected"] = True
                    detected[identifier]["contourArea"] = cv2.contourArea(maxContour)
                    detected[identifier]["side"] = "LEFT" if contourX < centreX else "RIGHT"
        
        return detected
    
    def getScreenshot(self):

        return self.rgbImage

    def onShutdown(self):

        cv2.destroyAllWindows()