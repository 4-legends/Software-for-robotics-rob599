#!/usr/bin/env python3

import rospy
import actionlib
import sys
from rob599_hw1.msg import robot_movement_seqAction, robot_movement_seqGoal, robot_movement_seqResult

class robot_seq_control_client:
	def __init__(self):
		self.client = actionlib.SimpleActionClient('robot_movement_seq', robot_movement_seqAction)


	def done_callback(self, status, result):

		if status == actionlib.GoalStatus.SUCCEEDED:
			rospy.loginfo('Reached goal with current distance {0}'.format(result.status))
		else:
			rospy.loginfo('Failed to reach goal with current distance {0}'.format(result.status))


	def active_callback(self):
		rospy.loginfo('Action is active')	


	def feedback_callback(self, feedback):
		rospy.loginfo('Feedback: {0}'.format(feedback.progress))


	def main(self):
		stopping_distance = 2
		
		self.client.wait_for_server()

		goal = robot_movement_seqGoal(number=stopping_distance)

		self.client.send_goal(goal, done_cb=self.done_callback, active_cb=self.active_callback, feedback_cb=self.feedback_callback)

		self.client.wait_for_result()



if __name__ == '__main__':

	rospy.init_node('robot_movement_seq_client')
	robot_seq_client = robot_seq_control_client()
	try:
		robot_seq_client.main()
	except KeyboardInterrupt:
		print("Shutting down")
	
