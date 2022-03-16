#!/usr/bin/env python
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
import math

class Navigation:

	waitTime = 60
	rotationSteps = 80

	'''
	Set up publisher and subscribers for navigation module
	'''
	def __init__(self, updateRate, rate):

		self.updateRate = updateRate
		self.rate = rate

		self.goalGiven = False

		self.rotationAmount = 0.0

		self.movementBase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.movementBase.wait_for_server()

		self.velocityPublisher = rospy.Publisher('mobile_base/commands/velocity', Twist)

	''' 
	Command robot to navigate to point (x,y) in world

	Returns once robot has reached (x,y)
	'''
	def navigateToPoint(self, point, angle):

		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = 'map'
		goal.target_pose.header.stamp = rospy.Time.now()

		point = Point(point[0], point[1], 0)
		quat = self.getQuarternion(angle)
		rotation = Quaternion(quat[0], quat[1], quat[2], quat[3])
		goal.target_pose.pose = Pose(point, rotation)

		self.movementBase.send_goal(goal)
		self.goalGiven = True

		success = self.movementBase.wait_for_result(rospy.Duration(Navigation.waitTime))

		state = self.movementBase.get_state()
		goalAchieved = False

		if success and state == GoalStatus.SUCCEEDED:
			return
		else:
			self.movementBase.cancel_goal()
			raise FailedNavigationException('failed to reach the navigation goal')

	'''
	Rotates robot 360 degrees at it's current position, returns true
	when still rotating and	false when rotation is stopped
	'''
	def rotateInPlace(self):

		offset = (2 * math.pi) / 10
		rotationStep = (2 * math.pi) / float(Navigation.rotationSteps)
		rotationVelocity = rotationStep * self.updateRate
		
		if (self.rotationAmount < (2 * math.pi) + offset):
			self.rotationAmount += rotationStep

			velocity = Twist()
			velocity.angular.z = rotationVelocity
			self.publishVelocityForTick(velocity)
			return True

		self.rotationAmount = 0.0
		self.publishVelocityForTick(Twist())
		return False

	'''
	Run when the node is shutdown
	'''	
	def onShutdown(self):

		if self.goalGiven:
			self.movementBase.cancel_goal()

	'''
	Construct quaternion from angle (radians) around z-axis
	'''
	def getQuarternion(self, theta):

		return 0, 0, np.sin(theta/2.0), np.cos(theta/2.0)

	'''
	Publish velocity for the duration of a tick (1 / self.updateRate)
	'''
	def publishVelocityForTick(self, velocity):

		self.velocityPublisher.publish(velocity)
		self.rate.sleep()

class FailedNavigationException(Exception):

	def __init__(self, message):

		super().__init__(message)

		

