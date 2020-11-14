#!/usr/bin/env python

import rospy
from rob599_hw1.srv import stopping_distance, stopping_distanceResponse

class Stopping_Distance_Monitor:
	def __init__(self):
		Distance = rospy.Service('stopping_distance', stopping_distance, self.stopping_distance)


	def stopping_distance(self, req):
		done = False
		if req.stop_dist > rospy.get_param('Range_max') or req.stop_dist < 0:
			rospy.logerr("Stopping Distance Invalid")
			return None
		else:
			rospy.set_param('Stopping_Distance', req.stop_dist)
			rospy.loginfo("Stopping Distance Changed to: {}".format(req.stop_dist))
			done = True
		response = stopping_distanceResponse()
		if done:
			response.Status = 1
		else:
			response.Status = 0
		return response



if __name__ == '__main__':
	rospy.init_node('stopping_distance_control_server', anonymous=True)
	stopping_distance = Stopping_Distance_Monitor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
