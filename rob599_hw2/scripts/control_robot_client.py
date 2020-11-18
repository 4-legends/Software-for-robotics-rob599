#!/usr/bin/env python

import rospy
import actionlib
import sys
from rob599_hw2.msg import control_robotAction, control_robotGoal, control_robotResult

class control_robot_client:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('control_robot', control_robotAction)


	def done_callback(self, status, result):

		if status == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Reached goal')
		else:
			rospy.loginfo('Failed to reach goal')


	def active_callback(self):
		rospy.loginfo('Action is active')	


	def feedback_callback(self, feedback):
		rospy.loginfo('Feedback: {0}'.format(feedback.progress))


	def main(self):
		goal_name = input('Enter goal position: ')
		
		self.client.wait_for_server()

		goal = control_robotGoal(name=goal_name)

		self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)

		self.client.wait_for_result()



if __name__ == '__main__':

	rospy.init_node('control_robot_client')
	robot_seq_client = control_robot_client()
	try:
		robot_seq_client.main()
	except KeyboardInterrupt:
		print("Shutting down")
	
