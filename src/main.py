#!/usr/bin/env python
import rospy
import numpy as np

from navigation import Navigation
from vision import Vision
from fileHandler import FileHandler

# Your code should go here. You can break your code into several files and
# include them in this file. You just need to make sure that your solution 
# can run by just running rosrun group_project main.py

class Robot:

	updateRate = 10

	filepath = 'world/input_points.yaml'

	sensitivity = 10
	colorRanges = {
		"green": {
			"lowerBound": np.array([60 - sensitivity, 100, 100]), 
			"upperBound": np.array([60 + sensitivity, 255, 255])
		},
		"red": {
			"lowerBound": np.array([0 - sensitivity, 100, 100]), 
			"upperBound": np.array([0 + sensitivity, 255, 255])
		}
	}	

	def __init__(self):

		self.rate = rospy.Rate(Robot.updateRate)

		self.navigation = Navigation(Robot.updateRate, self.rate)

		self.points = FileHandler.readPointsFile(Robot.filepath)

		self.vision = Vision()
		self.vision.setColorRanges(Robot.colorRanges)

		rospy.on_shutdown(self.onShutdown)

	def run(self):

		# Start by navigating to the entrance of room 1
		self.navigateToStartPoint()

		# Spin a full 360 degrees, detecting contours
		# to determine the color of the circle next to
		# room 1 entrance

		prominentColor = self.spinAndGetProminentColor()

		print(prominentColor)
	
	'''
	Run when the node is shutdown
	'''
	def onShutdown(self):

		self.navigation.onShutdown()

	'''
	Move to the entrance of room1
	'''
	def navigateToStartPoint(self):

		startPoint = self.points["room1"]["entrance"]
		self.navigation.navigateToPoint(startPoint, 0)

	'''
	Rotate 360 degrees on the spot, and return the color and
	size of the largest contour that was detected
	'''
	def spinAndGetProminentColor(self):

		largestContour = {"color": "green", "area": 0.0}

		while self.navigation.rotateInPlace():
			
			colors = {
				"green": {
					"detected": True,
					"contourArea": 20.0
				},
				"red": {
					"detected": False,
					"contourArea": 0.0
				}
			}

			#colors = self.detectColors()

			for color in colors:
				if colors[color]["detected"]:
					if colors[color]["contourArea"] > largestContour["area"]:
						largestContour["color"] = color
						largestContour["area"] = colors[color]["contourArea"]

		return largestContour

if __name__ == '__main__':

	rospy.init_node('cluedoRobot', anonymous=True)

	r = Robot()
	r.run()
