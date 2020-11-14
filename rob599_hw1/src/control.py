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
        self.laser_sub = rospy.Subscriber('/laser_scan', LaserScan, self.laser_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size= 1)
        Distance = rospy.Service('stopping_distance', stopping_distance, self.stopping_distance)
        self.marker_pub = marker_pub = rospy.Publisher('shortest_distance',Marker, queue_size=1)
        self.line_marker_pub = marker_pub = rospy.Publisher('shortest_distance_line',Marker, queue_size=1)
        self.stop_dist = float(rospy.get_param('Stopping_Distance'))
        self.action_in_progress = False
        self.max_range = 25
        self.server = actionlib.SimpleActionServer('robot_movement_seq', robot_movement_seqAction, self.action_callback, False)
        self.server.start()
        self.short_dist = self.max_range
	

    
    def velocity_controller(self, short_dist):
        vel = m.exp(abs(self.stop_dist - 0.1 - short_dist)/10) - 1
        if vel > 1:
            vel = 1
        return vel


    def stopping_distance(self, req):
        if not self.action_in_progress:
            done = False
            if req.stop_dist > self.max_range or req.stop_dist < 0:
                rospy.logerr("Stopping Distance Invalid")
                return None
            else:
                self.stop_dist = req.stop_dist
                rospy.set_param('Stopping_Distance', req.stop_dist)
                rospy.loginfo("Stopping Distance Changed to: {}".format(req.stop_dist))
                done = True
            response = stopping_distanceResponse()
            if done:
                response.Status = 1
            else:
                response.Status = 0
            return response
        else:
            return None


    def laser_callback(self, msg): # TODO: Make Code more general by tranforming everything to Base_link
        distance = msg.ranges
        shortest_dist_idx = 0
        self.max_range = msg.range_max
        local_short_dist = msg.range_max
        if self.action_in_progress:
            for i in range(len(distance)):
                if distance[i] < local_short_dist and distance[i] > 0:
                    local_short_dist = distance[i]
                    shortest_dist_idx = i
            rospy.loginfo("Current Shortest distance is: {}".format(local_short_dist))
            self.short_dist = local_short_dist
            if self.short_dist < self.stop_dist:
                velocity = 0
            else:
                velocity = self.velocity_controller(local_short_dist)
                
            vel_cmd = Twist()
            vel_cmd.linear.x  = velocity
            vel_cmd.linear.y  = 0
            vel_cmd.linear.z  = 0
            vel_cmd.angular.x = 0
            vel_cmd.angular.y = 0
            vel_cmd.angular.z = 0

            rospy.loginfo("Current Velocity is: {}".format(velocity))
            self.vel_pub.publish(vel_cmd)

            shortest_distance_marker = Marker()
            shortest_distance_marker.action = Marker.ADD
            shortest_distance_marker.type = Marker.TEXT_VIEW_FACING
            shortest_distance_marker.header.frame_id = msg.header.frame_id
            shortest_distance_marker.scale.z = 0.5
            shortest_distance_marker.color.a = 1
            shortest_distance_marker.color.r = 1.0
            shortest_distance_marker.color.g = 0.0
            shortest_distance_marker.color.b = 0.0    
            shortest_distance_marker.pose.position.x = distance[shortest_dist_idx]*np.cos(msg.angle_min+(msg.angle_increment*shortest_dist_idx))
            shortest_distance_marker.pose.position.y = distance[shortest_dist_idx]*np.sin(msg.angle_min+(msg.angle_increment*shortest_dist_idx))
            shortest_distance_marker.pose.orientation.w = 1

            multiline_str = 'Shortest Distance: %s'%str(local_short_dist) 
            shortest_distance_marker.text =  multiline_str 
            self.marker_pub.publish(shortest_distance_marker)
            
            shortest_distance_line_marker = Marker()
            shortest_distance_line_marker.action = Marker.ADD
            shortest_distance_line_marker.header.frame_id = msg.header.frame_id
            shortest_distance_line_marker.scale.x = 0.1
            shortest_distance_line_marker.color.a = 1
            shortest_distance_line_marker.color.r = 1.0
            shortest_distance_line_marker.color.g = 0.0
            shortest_distance_line_marker.color.b = 0.0    
            shortest_distance_line_marker.pose.orientation.w = 1
            shortest_distance_line_marker.type = Marker.LINE_STRIP
            shortest_distance_line_marker.points = [Point(0, 0, 0), Point(distance[shortest_dist_idx]*np.cos(msg.angle_min+(msg.angle_increment*shortest_dist_idx)), distance[shortest_dist_idx]*np.sin(msg.angle_min+(msg.angle_increment*shortest_dist_idx)), 0)]
            shortest_distance_line_marker.pose.orientation.w = 0
            shortest_distance_line_marker.pose.orientation.w = 0
            shortest_distance_line_marker.pose.orientation.w = 0
            shortest_distance_line_marker.pose.orientation.w = 1
            self.line_marker_pub.publish(shortest_distance_line_marker)

            



    def action_callback(self, goal):
        rospy.loginfo('Robot Sequence Action Server Started')
        self.action_in_progress = True
        
        if goal.number > self.max_range or goal.number < 0:
            rospy.logerr("Stopping Distance Invalid")
            return None
        else:
            self.stop_dist = goal.number

        while self.short_dist > self.stop_dist:
            self.server.publish_feedback(robot_movement_seqFeedback(progress=self.short_dist))

            if self.server.is_new_goal_available():
                result = 0
                self.server.set_preempted(robot_movement_seqResult(status=result))
                return

        result = 1
        self.server.set_succeeded(robot_movement_seqResult(status=result))
        self.action_in_progress = False
        rospy.loginfo("Robot Sequence Action Completed")

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    controller = Robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
