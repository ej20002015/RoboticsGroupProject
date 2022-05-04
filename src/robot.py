#!/usr/bin/env python
import rospy
import rospkg
import numpy as np

from navigation import Navigation
from vision import Vision
from fileHandler import FileHandler
import time

class FailedToFindRoomIndicator(Exception):

	pass

class Robot:

	updateRate = 10
	characterContourThreshold = {"min": 6500, "max": 10000}

	projectRoot = rospkg.RosPack().get_path("group_project")
	filepath = projectRoot + "/world/input_points.yaml"
	characterIdentifierFilepath = projectRoot + "/output/cluedo_character.txt"
	characterScreenshotFilepath = projectRoot + "/output/cluedo_character.png"

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
		"mustard": {
			"filepath": "cluedo_images/mustard.png",
			"lowerBound": np.array([20, 90, 0]),
			"upperBound": np.array([45, 150, 230])
		},
		"peacock": {
			"filepath": "cluedo_images/peacock.png",
			"lowerBound": np.array([100, 110, 0]),
			"upperBound": np.array([110, 150, 210])
		},
		"plum": {
			"filepath": "cluedo_images/plum.png",
			"lowerBound": np.array([150, 80, 0]),
			"upperBound": np.array([190, 150, 200])
		},
		"scarlet": {
			"filepath": "cluedo_images/scarlet.png",
			"lowerBound": np.array([-5, 160, 0]),
			"upperBound": np.array([5, 200, 200])
		}
	}

	def __init__(self):

		self.rate = rospy.Rate(Robot.updateRate)

		self.navigation = Navigation(Robot.updateRate, self.rate)

		self.points = FileHandler.readPointsFile(Robot.filepath) 

		self.roomMapping = {
			"greenRoom": "NONE", 
			"redRoom": "NONE"
		}

		self.vision = Vision(rgb=True, masks=True)
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
		
		# Perform inital sping to try and detect objects

		identifier = self.spinAndPursueObject()
		if identifier != "NONE":
			self.outputCharacterFiles(identifier)
		else:
			# Initial spin didn't find any objects, so move more

			stepInterval = 80
			stepCounter = 0

			while True:
				if self.navigation.moveForwardWithCollision(0.25):
					self.navigation.rotateByAngle(30, "CLOCKWISE", 5)
				else:
					stepCounter += 1

				if (stepCounter % stepInterval == 0):
					identifier = self.spinAndPursueObject()
					if identifier != "NONE":
						self.outputCharacterFiles(identifier)

	'''
	Run when the node is shutdown
	'''
	def onShutdown(self):

		self.navigation.onShutdown()
		self.vision.onShutdown()

	def outputCharacterFiles(self, identifier):

		image = self.vision.getScreenshot()
		FileHandler.writeTextFile(Robot.characterIdentifierFilepath, identifier)
		FileHandler.writeScreenshotFile(Robot.characterScreenshotFilepath, image)

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

	def spinAndPursueObject(self):

		# Attempt to identify the cluedo character by spinning 360 degrees in the center of the room
		obj = self.spinAndFindObject()

		if (obj["identifier"] != "NONE"):
			self.centreAndPursueObject(obj)
			return obj["identifier"]
		
		return "NONE"

	'''
	Rotate up to 360 degrees on the spot and return the 
	first detected object
	'''
	def spinAndFindObject(self):

		obj = {"identifier": "NONE", "area": 0.0, "side": "NONE"}

		while self.navigation.rotateInPlace(speedScale = 0.8):
			
			objects = self.vision.detectPresenceOfObjects()

			for identifier, details in objects.items():
				if details["detected"]:
					obj["identifier"] = identifier
					obj["area"] = details["contourArea"]
					obj["side"] = details["side"]
					return obj
		
		return obj

	'''
	Centres the object in the centre of the robot's vision, and
	navigates towards the object 
	'''
	def centreAndPursueObject(self, obj):

		currentSide = obj["side"]

		stepInterval = 20
		movementSpeedScale = 0.2
		rotationSpeedScale = 0.1

		stepCounter = 0

		while True:
			
			if (stepCounter % stepInterval == 0):
				rotationDirection = "CLOCKWISE" if currentSide == "RIGHT" else "ANTI_CLOCKWISE"

				while self.navigation.rotateInPlace(rotationDirection, rotationSpeedScale):

					objects = self.vision.detectPresenceOfObjects()
					if objects[obj["identifier"]]["side"] != currentSide and objects[obj["identifier"]]["side"] != "NONE":
						currentSide = objects[obj["identifier"]]["side"]
						break
			
			stepCounter += 1

			objects = self.vision.detectPresenceOfObjects()
			if objects[obj["identifier"]]["contourArea"] < Robot.characterContourThreshold["min"]:
				# Move forward
				self.navigation.moveStraight("FORWARD", movementSpeedScale)
			elif objects[obj["identifier"]]["contourArea"] > Robot.characterContourThreshold["max"]:
				# Move backward
				self.navigation.moveStraight("BACKWARD", movementSpeedScale)
			else:
				self.navigation.stop()
				break
			


