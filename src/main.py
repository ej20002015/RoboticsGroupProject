#!/usr/bin/env python
import rospy
from navigation import Navigation
from fileHandler import FileHandler

# Your code should go here. You can break your code into several files and
# include them in this file. You just need to make sure that your solution 
# can run by just running rosrun group_project main.py

class Robot:

	filepath = 'world/input_points.yaml'	

	def __init__(self):

		self.navigation = Navigation()

		self.points = FileHandler.readPointsFile(Robot.filepath)

		rospy.on_shutdown(self.onShutdown)

	def run(self):

		self.navigation.navigateToPoint(self.points['room2']['centre'], 0)

	def onShutdown(self):

		self.navigation.onShutdown()

if __name__ == '__main__':

	rospy.init_node('cluedoRobot', anonymous=True)

	r = Robot()
	r.run()
