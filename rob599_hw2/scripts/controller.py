#!/usr/bin/env python

import rospy
import numpy as np
import csv 
import tf
import actionlib
from rob599_hw2.srv import memorize_position, memorize_positionResponse
from nav_msgs.msg import Odometry
from rob599_hw2.msg import control_robotAction, control_robotGoal, control_robotFeedback, control_robotResult
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback,MoveBaseActionResult
#from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Robot_controller:
    def __init__(self):
        
        self.tf_listener = tf.TransformListener()
        self.odom_msg = np.zeros([2,4])
        self.goal_pose = MoveBaseActionGoal()
        self.frame_id = ''
        self.known_location = {}
        self.goal = 1
        self.feedback = 0
        self.result = False
        Distance = rospy.Service('memorize_position', memorize_position, self.memorize_position_func)
        self.csv_writer = open('/home/graspinglab/catkin_ws/src/rob599_hw2/config/known_location.txt', 'w+')
        self.csv_writer.write('Name, Frame_id, Child_frame_id, [Pose[x,y,z], Orientation[x,y,z,w]]\n')
        # self.marker_pub = marker_pub = rospy.Publisher('shortest_distance',Marker, queue_size=1)
        # self.line_marker_pub = marker_pub = rospy.Publisher('shortest_distance_line',Marker, queue_size=1)
        self.goal_pub = rospy.Publisher('move_base/goal',MoveBaseActionGoal, queue_size=1)
        self.server = actionlib.SimpleActionServer('control_robot', control_robotAction, self.action_callback, False)
        self.server.start()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.status_sub = rospy.Subscriber('move_base/feedback', MoveBaseActionFeedback, self.status_callback)
        self.result_sub = rospy.Subscriber('move_base/result', MoveBaseActionResult, self.result_callback)
         
	

    def memorize_position_func(self, req):
        self.known_location[req.name] = self.goal_pose
        self.csv_writer.write("{}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(req.name, self.frame_id, self.odom_msg[0,0], self.odom_msg[0,1], self.odom_msg[0,2], self.odom_msg[1,0], self.odom_msg[1,1], self.odom_msg[1,2], self.odom_msg[1,3]))
        return True


    def odom_callback(self, msg):
        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.stamp = rospy.Time(0)
        goal_pose_msg.header.frame_id = msg.header.frame_id
        goal_pose_msg.pose.position.x = msg.pose.pose.position.x
        goal_pose_msg.pose.position.y = msg.pose.pose.position.y
        goal_pose_msg.pose.position.z = msg.pose.pose.position.z
        goal_pose_msg.pose.orientation.x = msg.pose.pose.orientation.x
        goal_pose_msg.pose.orientation.y = msg.pose.pose.orientation.y
        goal_pose_msg.pose.orientation.z = msg.pose.pose.orientation.z
        goal_pose_msg.pose.orientation.w = msg.pose.pose.orientation.w
        self.tf_listener.waitForTransform("/map", msg.header.frame_id, goal_pose_msg.header.stamp, rospy.Duration(0.5))
        goal_pose_local = self.tf_listener.transformPose("/map", goal_pose_msg)
        self.frame_id = goal_pose_local.header.frame_id
        self.odom_msg[0,0] = goal_pose_local.pose.position.x
        self.odom_msg[0,1] = goal_pose_local.pose.position.y
        self.odom_msg[0,2] = goal_pose_local.pose.position.z
        self.odom_msg[1,0] = goal_pose_local.pose.orientation.x
        self.odom_msg[1,1] = goal_pose_local.pose.orientation.y
        self.odom_msg[1,2] = goal_pose_local.pose.orientation.z
        self.odom_msg[1,2] = goal_pose_local.pose.orientation.w
        self.goal_pose.header = goal_pose_local.header
        self.goal_pose.goal.target_pose = goal_pose_local
        print(self.goal_pose)


    def status_callback(self, msg):
        self.feedback = msg

    def result_callback(self, msg):
        self.result = True


    def action_callback(self, goal):
        rospy.loginfo('Control Robot Action Server Started')
        
        try: 
            goal = self.known_location[goal.name] 
        except:
            rospy.logerr("Invalid name entered")
            return None
        self.goal_pose.goal_id = self.goal
        self.goal += 1
        while not self.result:
            self.server.publish_feedback(control_robotFeedback(progress=self.feedback))

            if self.server.is_new_goal_available():
                result = 0
                self.server.set_preempted(control_robotResult(status=result))
                return
        self.result = False
        result = 1
        self.server.set_succeeded(control_robotResult(status=result))
        rospy.loginfo("Control Robot Action Completed") 

if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    controller = Robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")