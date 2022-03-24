#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import os

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Vision:

    distanceThreshold = 55
    histogramSensitivity = 0.75
    histogramBins = 10

    def __init__(self, **kwargs):

        self.displayRGB = kwargs.get('rgb', False)
        self.displayKP = kwargs.get('kp', False)
        self.displayMasks = kwargs.get('masks', False)

        # colors we want to detect
        self.colors = {}
        # Filepaths and identifiers of the images containing the objects we want to detect
        self.objectKeypoints = {}

        self.detector = cv2.ORB(nfeatures=1000,patchSize=100)
        self.matcher = cv2.BFMatcher(normType=cv2.NORM_HAMMING, crossCheck=False)
        self.classifier = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

        self.rgbImage = None
        self.initializeWindows()

        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.update)

    def initializeWindows(self):

        cv2.destroyAllWindows()
        cv2.namedWindow('rgb')
        cv2.namedWindow('kp')
        cv2.namedWindow('grey')
        cv2.namedWindow('woman')

        if self.displayMasks:
            for color in self.colors.keys() : cv2.namedWindow('rgbMask-' + color)

    def update(self, image):

        self.rgbImage = self.bridge.imgmsg_to_cv2(image, 'bgr8')

        if self.displayRGB:
            cv2.imshow('rgb', self.rgbImage)
            cv2.waitKey(3)

        if self.displayKP:
            
            keypoints = self.detector.detect(self.rgbImage)
            keypoints_img = cv2.drawKeypoints(self.rgbImage, keypoints, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DEFAULT)
            cv2.imshow('kp', keypoints_img)
            cv2.waitKey(3)

    def setColorRanges(self, colorRanges):

        self.colors = colorRanges
        self.initializeWindows()

    def setObjectRecognitionImagePaths(self, objectRecognitionImagePaths):

        self.objectKeypoints.clear()
        for identifier, path in objectRecognitionImagePaths.items():
            img = cv2.imread(path)

            hsvImage = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            colorHistogram = cv2.calcHist([hsvImage.astype('float32')], channels=[0], mask=None, histSize=[Vision.histogramBins], ranges=[0, 256]).flatten()
            
            middle = np.argmax(colorHistogram)
            lowerValue = middle - Vision.histogramSensitivity * Vision.histogramBins
            upperValue = middle + Vision.histogramSensitivity * Vision.histogramBins

            size = hsvImage.shape
            hsvImageReshape = hsvImage.reshape(size[0]*size[1], 3)
            pixelsInMask = [(pixel[1], pixel[2]) for pixel in hsvImageReshape if pixel[0] < upperValue and pixel[0] > lowerValue]

            saturationValues = [pixel[0] for pixel in pixelsInMask]
            meanSaturation = np.average(saturationValues)
            stdSaturation = np.std(saturationValues)
            
            valueValues = [pixel[1] for pixel in pixelsInMask]
            meanValue = np.average(valueValues)
            stdValue = np.std(valueValues)

            print('Saturation: ' + str(meanSaturation - 1 * stdSaturation) + ' - ' + str(meanSaturation + 1 * stdSaturation))
            print('Values: ' + str(meanValue - 1 * stdValue) + ' - ' + str(meanValue + 1 * stdValue))

            lower = np.array([lowerValue, meanSaturation - 1 * stdSaturation, meanValue - 1 * stdValue])
            upper = np.array([upperValue, meanSaturation + 1 * stdSaturation, meanValue + 1 * stdValue])

            mask = cv2.inRange(hsvImage, lower, upper)
            maskedImage = cv2.bitwise_and(img, img, mask=mask)
            cv2.imshow('woman', maskedImage)

            kp, des = self.detector.detectAndCompute(img, None)
            self.objectKeypoints[identifier] = {
                "kp": kp,
                "des": des,
                "mask": (lower, upper)
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

        grayImg = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2GRAY)
        faces = self.classifier.detectMultiScale(grayImg, 1.1, 5)

        for (x, y, w, h) in faces:
            cv2.rectangle(grayImg, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.imshow('grey', grayImg)

        if len(faces) == 0 or self.rgbImage is None: 
            return False
        
        hsvImage = cv2.cvtColor(self.rgbImage, cv2.COLOR_BGR2HSV)
        for identifier, prototype in self.objectKeypoints.items():

            mask = cv2.inRange(hsvImage, prototype['mask'][0], prototype['mask'][1])
            maskedImage = cv2.bitwise_and(self.rgbImage, self.rgbImage, mask=mask)
            cv2.imshow('grey', maskedImage)

            contours = cv2.findContours(mask, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)[0]
            if len(contours) > 0:
                maxContour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(maxContour) > 20:
                    return True

        return False
                    
        # if self.rgbImage is not None:

        #     videoFeedKeypoints, videoFeedDescriptors = self.detector.detectAndCompute(self.rgbImage, None)

        #     for identifier, prototype in self.objectKeypoints.items():

        #         pass

        #         # allMatches = self.matcher.knnMatch(prototype['des'], videoFeedDescriptors, k=2)
        #         # func = lambda distances : np.average([m.distance for m in distances])
        #         # aggregatedMatches = sorted([func(neighbours) for neighbours in allMatches])
        #         # aggregatedMatches = aggregatedMatches[:int(len(aggregatedMatches) * 0.1)]
        #         # totalMatchScore = sum(aggregatedMatches)

        #         # goodMatches = [match for match in aggregatedMatches if match < Vision.distanceThreshold]

        #         # percentageMatched = float(len(goodMatches)) / len(prototype['des'])
        #         # results[identifier] = totalMatchScore

        # return results

    def onShutdown(self):

        cv2.destroyAllWindows()