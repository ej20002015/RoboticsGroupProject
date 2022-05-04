#!/usr/bin/env python
import rospy
import yaml
import cv2

class FileHandler:

    @staticmethod
    def writeTextFile(filePath, string):
        with open(filePath, "w") as file:
            file.write(string)

    @staticmethod
    def writeScreenshotFile(filePath, image):
        cv2.imwrite(filePath, image)

    @staticmethod
    def readPointsFile(filePath):
        
        with open(filePath, 'r') as file:
            points = yaml.safe_load(file)

        return {
            'room1': {
                'entrance': tuple(points['room1_entrance_xy']), 
                'centre': tuple(points['room1_centre_xy'])
            },
            'room2': {
                'entrance': tuple(points['room2_entrance_xy']), 
                'centre': tuple(points['room2_centre_xy'])
            }
        }
