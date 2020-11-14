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


	def laser_callback(self, msg): # TODO: Make Code more general by tranforming everything to Base_link  
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

		if abs(msg.angle_min) < abs(msg.angle_max):
			i = int(round((len(distance)-1) * (1-abs(msg.angle_max)/(abs(msg.angle_min)+abs(msg.angle_max)))))
		elif abs(msg.angle_max) < abs(msg.angle_min):
			i = int(round((len(distance)-1) * ((abs(msg.angle_min)/(abs(msg.angle_min)+abs(msg.angle_max))))))
		else:
			i = int(round((len(distance) - 1)/2))
		
		for m in range(len(distance)):
			if distance[m] > msg.range_max:
				distance[m] = msg.range_max
		
		robot_half_width = rospy.get_param('Robot_Half_Width')

		#Code detecting Triangle in front of robot with projecting robot width on the front surface
		# j = 1
		# k = 1 
		# loop_variable = 1
		# done = 0
		# while  done == 0:
		# 	if np.sqrt(-(np.cos(angle*k)*2*distance[i]*distance[i-k])+distance[i-k]**2 + distance[i]**2) <= robot_half_width:
		# 		k+= 1

		# 	if np.sqrt(-(np.cos(angle*j)*2*distance[i]*distance[i+j])+distance[i+j]**2 + distance[i]**2) <= robot_half_width:
		# 		j += 1

		# 	if loop_variable >= (len(distance)/2):
		# 		done = 1

		# 	if (np.sqrt(-(np.cos(angle*j)*2*distance[i]*distance[i+j])+distance[i+j]**2 + distance[i]**2) > robot_half_width) and (np.sqrt(-(np.cos(angle*j)*2*distance[i]*distance[i-k])+distance[i-k]**2 + distance[i]**2) > robot_half_width):
		# 		done = 1

		# 	loop_variable+= 1

		# min_angle = -k*angle
		# max_angle = j*angle
		# distance = distance[i-k:i+j+1]
		
		#Projecting each laser scan point on Center line and finding distance of each point with respect to center line 
		filtered_values_ranges = []
		center = np.array([0,0])
		not_in_front = 0
		for scan in range(len(distance)):
			#front = np.array([distance[i]*np.cos(min_angle+(angle*scan)), distance[i]*np.sin(min_angle+(angle*scan))])
			side = np.array([distance[scan]*np.cos(min_angle+(angle*scan)), distance[scan]*np.sin(min_angle+(angle*scan))])
			if side[0] < 0:
				filtered_values_ranges.append(not_in_front)
				continue 
 			
			# ap = side - center
			# ab = front - center
			# project_point = center + (np.dot(ap, ab)/np.dot(ab,ab)) * ab
			# dist = np.sqrt((side[0]-project_point[0])**2+(side[1]-project_point[1])**2)

			if abs(side[1]) < robot_half_width:
				filtered_values_ranges.append(distance[scan])
			else:
				filtered_values_ranges.append(not_in_front)
	
		filtered_values.ranges = filtered_values_ranges
		filtered_values.angle_min = min_angle
		filtered_values.angle_max = max_angle
		self.scan_pub.publish(filtered_values)


if __name__ == '__main__':
	rospy.init_node('filter', anonymous=True)
	laser_filter = Laser_Filter()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
