#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
import numpy as np
import math as m 
from sensor_msgs.msg import LaserScan
from rob599_hw1.srv import stopping_distance, stopping_distanceResponse
import actionlib
from rob599_hw1.msg import robot_movement_seqAction, robot_movement_seqGoal, robot_movement_seqFeedback, robot_movement_seqResult
from visualization_msgs.msg import Marker


#######################################
# Laser Scan: 
#   Header: Seq, Stamp, frame_id
#   Angle_min, Angle_max, Angle_Increment, Time_Increment
#   Scan time, range_min, range_max, ranges, intensities 
#
# Twist:
#   Linear: x, y, z
#   Angular: x, y, z
#######################################

class Robot_controller:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
    
        self.line_marker_pub = marker_pub = rospy.Publisher('/angle_marker',Marker, queue_size=1)



    def laser_callback(self, msg): # TODO: Make Code more general by tranforming everything to Base_link 
        distance = msg.ranges
        shortest_dist_idx = 0
        local_short_dist = msg.range_max

        if abs(msg.angle_min) < abs(msg.angle_max):
            i = int(round((len(distance)-1) * (1-abs(msg.angle_max)/(abs(msg.angle_min)+abs(msg.angle_max)))))
        elif abs(msg.angle_max) < abs(msg.angle_min):
            i = int(round((len(distance)-1) * ((abs(msg.angle_min)/(abs(msg.angle_min)+abs(msg.angle_max))))))
        else:
            i = int(round((len(distance) - 1)/2))

        for scan in range(len(distance)):
            if distance[scan] < local_short_dist and distance[scan] > 0:
                local_short_dist = distance[scan]
                shortest_dist_idx = scan
   
        theta = np.pi/6
        center_scan_id = int(len(msg.ranges)/2)
        center_range = msg.ranges[center_scan_id]
        angle_array = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        theta_idx = (np.abs(angle_array - theta)).argmin()
        ray2 = msg.ranges[theta_idx]
        len_c = np.sqrt(center_range**2 + ray2**2 - (2*center_range*ray2*np.cos(theta)))
        angle = np.pi/2 - np.arcsin(ray2*np.sin(theta)/len_c)

        shortest_distance_marker = Marker()
        shortest_distance_marker.action = Marker.ADD
        shortest_distance_marker.type = Marker.TEXT_VIEW_FACING
        shortest_distance_marker.header.frame_id = msg.header.frame_id
        shortest_distance_marker.scale.z = 0.1
        shortest_distance_marker.color.a = 1
        shortest_distance_marker.color.r = 1.0
        shortest_distance_marker.color.g = 1.0
        shortest_distance_marker.color.b = 0.0    
        shortest_distance_marker.pose.position.x = distance[center_scan_id]
        shortest_distance_marker.pose.position.y = 0
        shortest_distance_marker.pose.orientation.w = 1

        multiline_str = 'Angle of Robot with respect to wall: %s'%str(angle) 
        shortest_distance_marker.text =  multiline_str 
        self.line_marker_pub.publish(shortest_distance_marker)


if __name__ == '__main__':
    rospy.init_node('line_fit', anonymous=True)
    controller = Robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
