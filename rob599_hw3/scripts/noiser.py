#!/usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
#######################################
# Laser Scan: 
# 	Header: Seq, Stamp, frame_id
# 	Angle_min, Angle_max, Angle_Increment, Time_Increment
#	Scan time, range_min, range_max, ranges, intensities 
#######################################

class Noise_class:
	def __init__(self):
		#rospy.on_shutdown(self.save_csv)
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
		self.scan_pub = rospy.Publisher('/gaus_err_laser_scan', LaserScan, queue_size= 1)


	def laser_callback(self, msg):
		filtered_values = LaserScan()
		distance = np.array(msg.ranges)
		filtered_values.header = msg.header
		filtered_values.angle_increment = msg.angle_increment
		filtered_values.time_increment = msg.time_increment
		filtered_values.scan_time = msg.scan_time
		filtered_values.range_min = msg.range_min
		filtered_values.range_max = msg.range_max
		filtered_values.intensities = msg.intensities
		angle = filtered_values.angle_increment
		min_angle = msg.angle_min
		max_angle = msg.angle_max
		
		laser_noise_variance = rospy.get_param('laser_noise_variance')
		if laser_noise_variance <= 0:
			laser_noise_variance = 0.1
		
		filtered_values_ranges = np.zeros(len(distance))
		noise_values_ranges = np.random.normal(loc = 0, scale=laser_noise_variance, size=len(distance))

		for i in range(len(distance)):
			filtered_values_ranges[i] = noise_values_ranges[i]+distance[i]

		filtered_values.ranges = filtered_values_ranges
		filtered_values.angle_min = min_angle
		filtered_values.angle_max = max_angle
		self.scan_pub.publish(filtered_values)


if __name__ == '__main__':
	rospy.init_node('noiser', anonymous=True)
	noisy = Noise_class()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
