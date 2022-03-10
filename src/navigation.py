#!/usr/bin/env python
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion

class Navigation:

	waitTime = 60

	'''
	Set up publisher and subscribers for navigation module
	'''
	def __init__(self):

		self.goalGiven = False

		self.movementBase = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self.movementBase.wait_for_server()

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
	Run on shutdown
	'''	
	def onShutdown(self):

		if self.goalGiven:
			self.movementBase.cancel_goal()

	'''
	Construct quaternion from angle (radians) around z-axis
	'''
	def getQuarternion(self, theta):

		return 0, 0, np.sin(theta/2.0), np.cos(theta/2.0)

class FailedNavigationException(Exception):

	def __init__(self, message):

		super().__init__(message)

		

