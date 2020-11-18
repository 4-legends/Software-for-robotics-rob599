#!/usr/bin/env python

import rospy
import actionlib
import sys
from rob599_hw2.msg import patrolAction, patrolGoal, patrolResult
from std_msgs.msg import Empty

class patrol_client:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('patrol', patrolAction)


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
		goal = Empty()

		self.client.wait_for_server()

		self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)

		self.client.wait_for_result()



if __name__ == '__main__':

	rospy.init_node('patrol_client')
	robot_seq_client = patrol_client()
	try:
		robot_seq_client.main()
	except KeyboardInterrupt:
		print("Shutting down")
	
