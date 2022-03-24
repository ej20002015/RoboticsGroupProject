#!/usr/bin/env python
import rospy
import numpy as np

from navigation import Navigation
from vision import Vision
from fileHandler import FileHandler

class FailedToFindRoomIndicator(Exception):

	pass

class Robot:

	updateRate = 10

	filepath = 'world/input_points.yaml'

	sensitivity = 20
	colorRanges = {
		"green": {
			"lowerBound": np.array([60 - sensitivity, 50, 50]), 
			"upperBound": np.array([60 + sensitivity, 255, 255])
		},
		"red": {
			"lowerBound": np.array([0, 100, 100]),
			"upperBound": np.array([0, 255, 255])
		}
	}

	objectRecognitionImagePaths = {
		"mustard": "cluedo_images/mustard.png",
		# "peacock": "cluedo_images/peacock.png",
		# "plum":    "cluedo_images/plum.png",
		#"scarlet": "cluedo_images/scarlet.png"
	}

	def __init__(self):

		self.rate = rospy.Rate(Robot.updateRate)

		self.navigation = Navigation(Robot.updateRate, self.rate)

		self.points = FileHandler.readPointsFile(Robot.filepath) 

		self.roomMapping = {
			"greenRoom": "NONE", 
			"redRoom": "NONE"
		}

		self.vision = Vision(rgb=True, kp=True)
		self.vision.setColorRanges(Robot.colorRanges)
		self.vision.setObjectRecognitionImagePaths(Robot.objectRecognitionImagePaths)

		rospy.on_shutdown(self.onShutdown)

	def run(self):

		# Start by navigating to the entrance of room 1
		self.navigateToStartPoint()

		# Spin a full 360 degrees, detecting contours
		# to determine which room is green, and which
		# is red

		prominentColor = self.spinAndGetProminentColor()

		if prominentColor == "green":
			self.roomMapping["greenRoom"] = "room1"
			self.roomMapping["redRoom"] = "room2"
		elif prominentColor == "red":
			self.roomMapping["greenRoom"] = "room2"
			self.roomMapping["redRoom"] = "room1"
		else:
			raise FailedToFindRoomIndicator("Was looking for " + str(self.colorRanges.keys()) +
											" - Failed to find any of these colors")

		# Move into the green room
		self.navigation.navigateToPoint(self.points[self.roomMapping["greenRoom"]]["centre"], 0)

		# Spin a full 360 degrees, detecting character
		# keypoints to determine if we see a character, and
		# if so, their identity

		minimums = None

		while self.navigation.rotateInPlace():
			
			results = self.vision.detectPresenceOfObjects()
			# if minimums is None : minimums = results
			# else:
			# 	for key in results.keys():
			# 		if results[key] <= minimums[key]: 
			# 			minimums[key] = results[key]

			print(results)

	'''
	Run when the node is shutdown
	'''
	def onShutdown(self):

		self.navigation.onShutdown()
		self.vision.onShutdown()

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

		largestContour = {"color": "NONE", "area": 0.0}

		while self.navigation.rotateInPlace():

			colors = self.vision.detectColors()

			for color in colors:
				if colors[color]["detected"]:
					if colors[color]["contourArea"] > largestContour["area"]:
						largestContour["color"] = color
						largestContour["area"] = colors[color]["contourArea"]

		return largestContour["color"]