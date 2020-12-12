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

class Laser_Filter:
	def __init__(self):
		#rospy.on_shutdown(self.save_csv)
		self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
		self.scan_pub = rospy.Publisher('/laser_scan', LaserScan, queue_size= 1)


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
		
		median_filter_size = rospy.get_param('median_filter_size')
		if median_filter_size < 1:
			median_filter_size = 1
		elif median_filter_size > len(distance)/2 - 1:
			median_filter_size = int(len(distance)/2 - 1)

		filtered_values_ranges = np.zeros(len(distance))

		for i in range(len(distance) - median_filter_size - 1):
			if i < median_filter_size:
				filtered_values_ranges[i] = 0
			else:
				filtered_values_ranges[i] = np.median(distance[(i - median_filter_size):(i + median_filter_size+1)])
				if filtered_values_ranges[i] > msg.range_max or filtered_values_ranges[i] < 0:
					filtered_values_ranges[i] = 0

		filtered_values.ranges = filtered_values_ranges
		filtered_values.angle_min = min_angle
		filtered_values.angle_max = max_angle
		self.scan_pub.publish(filtered_values)


if __name__ == '__main__':
	rospy.init_node('median_filter', anonymous=True)
	laser_filter = Laser_Filter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
