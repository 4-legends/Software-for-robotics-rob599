#!/usr/bin/env python

import rospy
from rob599_hw1.srv import stopping_distance, stopping_distanceRequest

class Stopping_Distance_Client:
	def __init__(self):
		self.stop_dist_client = rospy.ServiceProxy('stopping_distance', stopping_distance)


	def main(self):
		req = stopping_distanceRequest()
		req.stop_dist = 3
		resp = self.stop_dist_client(req)
		

if __name__ == '__main__':
	rospy.init_node('stopping_distance_client', anonymous=True)
	stopping_distance = Stopping_Distance_Client()
	try:
		stopping_distance.main()
	except KeyboardInterrupt:
		print("Shutting down")
