#!/usr/bin/env python
import rospy
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from kobuki_msgs.msg import BumperEvent
import math


class FailedNavigationException(Exception):

	pass

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
		self.bumperSubscriber = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.bumperEventCallback)

		self.bumperEvent = None

	def bumperEventCallback(self, data):

		self.bumperEvent = data

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
			raise FailedNavigationException('Failed to reach the navigation goal')

	'''
	Rotates robot 360 degrees at it's current position, returns true
	when still rotating and	false when rotation is stopped
	'''
	def rotateInPlace(self, direction = "ANTI_CLOCKWISE", speedScale = 1.0):

		sign = 1 if direction == "ANTI_CLOCKWISE" else -1
		offset = (2 * math.pi) / 20
		rotationStep = (2 * math.pi) / (float(Navigation.rotationSteps) * (1.0 / speedScale))
		rotationVelocity = sign * rotationStep * self.updateRate
		
		if (abs(self.rotationAmount) < (2 * math.pi) + offset):
			self.rotationAmount += sign * rotationStep

			velocity = Twist()
			velocity.angular.z = rotationVelocity
			self.publishVelocityForTick(velocity)
			return True

		self.rotationAmount = 0.0
		self.stop()
		return False

	def rotateByAngle(self, angle, direction = "ANTI_CLOCKWISE", speedScale = 1.0):
		
		sign = 1 if direction == "ANTI_CLOCKWISE" else -1
		targetAngle = (2 * math.pi) * (angle / 360.0)
		offset = targetAngle / 20
		rotationStep = targetAngle / (float(Navigation.rotationSteps) * (1.0 / speedScale))
		rotationVelocity =  sign * rotationStep * self.updateRate
		rotationAmount = 0.0
		
		while (abs(rotationAmount) < targetAngle + offset):
			rotationAmount += sign * rotationStep

			velocity = Twist()
			velocity.angular.z = rotationVelocity
			self.publishVelocityForTick(velocity)

	def moveStraight(self, direction, speedScale = 1.0):

		sign = 1 if direction == "FORWARD" else -1
		velocity = Twist()
		velocity.linear.x = 1 * speedScale * sign
		self.publishVelocityForTick(velocity)

	def moveForwardWithCollision(self, speedScale = 1.0):

		reverseMovementIterations = 5
		forwardVelocity = Twist()
		forwardVelocity.linear.x = 1 * speedScale
		self.publishVelocityForTick(forwardVelocity)

		# Move back a certian distance if collision is detected

		if self.bumperEvent is not None:

			if self.bumperEvent.state == BumperEvent.PRESSED:

				print(self.bumperEvent)

				reverseVelocity = Twist()
				reverseVelocity.linear.x = -1 * speedScale
				for i in range(reverseMovementIterations):
					print("Going back")
					self.publishVelocityForTick(reverseVelocity)
				return True
		
		return False


	def stop(self):

		self.publishVelocityForTick(Twist())

	def getRotationAmount(self):
		
		return self.rotationAmount

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

		

