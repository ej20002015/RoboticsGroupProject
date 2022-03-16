#!/usr/bin/env python
import rospy

from robot import Robot

# Your code should go here. You can break your code into several files and
# include them in this file. You just need to make sure that your solution 
# can run by just running rosrun group_project main.py

if __name__ == '__main__':

	rospy.init_node('cluedoRobot', anonymous=True)

	r = Robot()
	r.run()
