#!/usr/bin/env python

import rospy
from rob599_hw2.srv import file_position, file_positionRequest

class File_Client:
	def __init__(self):
		self.file_client = rospy.ServiceProxy('file_position', file_position)


	def main(self):
		req = file_positionRequest()
		req.name = input('Enter save or load: ')
		resp = self.file_client(req)
		print(resp)
		

if __name__ == '__main__':
	rospy.init_node('file_position_client', anonymous=True)
	file_position = File_Client()
	try:
		file_position.main()
	except KeyboardInterrupt:
		print("Shutting down")
