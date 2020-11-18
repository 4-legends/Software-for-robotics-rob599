#!/usr/bin/env python

import rospy
from rob599_hw2.srv import memorize_position, memorize_positionRequest

class Memory_Client:
	def __init__(self):
		self.memory_client = rospy.ServiceProxy('memorize_position', memorize_position)


	def main(self):
		req = memorize_positionRequest()
		req.name = input('Enter Location name: ')
		resp = self.memory_client(req)
		print(resp)
		

if __name__ == '__main__':
	rospy.init_node('memory_client', anonymous=True)
	memorize = Memory_Client()
	try:
		memorize.main()
	except KeyboardInterrupt:
		print("Shutting down")
