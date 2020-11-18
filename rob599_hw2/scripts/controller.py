#!/usr/bin/env python

import rospy
import os
import csv 
import actionlib
from rob599_hw2.srv import memorize_position, file_position
from sensor_msgs.msg import Image
from rob599_hw2.msg import control_robotAction, control_robotGoal, control_robotFeedback, control_robotResult
from rob599_hw2.msg import patrolAction, patrolGoal, patrolResult, patrolFeedback
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rosbag


class Robot_controller:
    def __init__(self):
        self.known_location = {}
        self.current_img = Image()
        self.heading = PoseWithCovarianceStamped()
        self.filepath = os.path.dirname(os.path.realpath(__file__))
        Current_loc = rospy.Service('memorize_position', memorize_position, self.memorize_position_func)
        file = rospy.Service('file_position', file_position, self.file_position_func)
        self.server = actionlib.SimpleActionServer('control_robot', control_robotAction, self.action_callback, False)
        self.patrol_server = actionlib.SimpleActionServer('patrol', patrolAction, self.patrol_action_callback, False)
        self.server.start()
        self.patrol_server.start()
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.image_sub = rospy.Subscriber('/head_camera/rgb/image_raw', Image, self.image_callback)
        

    def memorize_position_func(self, req):
        self.known_location[req.name] = self.heading
        return True


    def file_position_func(self, req):
        if req.name == 'save':
            with open(self.filepath+'/location/known_location.txt', 'a') as self.csv_writer:
                for key, value in enumerate(self.known_location):
                    goal = self.known_location[value]
                    self.csv_writer.write("{}, {}, {}, {}, {}, {}, {}, {}, {}\n".format(value, goal.header.frame_id, goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z, goal.pose.pose.orientation.x, goal.pose.pose.orientation.y, goal.pose.pose.orientation.z, goal.pose.pose.orientation.w))
        elif req.name == 'load':
            data = []
            with open(self.filepath+'/location/known_location.txt') as csvfile:
                checker=csvfile.readline()
                if ',' in checker:
                    delim=','
                else:
                    delim=' '
                reader = csv.reader(csvfile, delimiter=delim)
                for i in reader:
                    if i[0] != 'Name':
                        to_save = PoseWithCovarianceStamped()
                        name = i[0]
                        to_save.header.frame_id = 'map'
                        to_save.pose.pose.position.x = float(i[2])
                        to_save.pose.pose.position.y = float(i[3])
                        to_save.pose.pose.position.z = float(i[4])
                        to_save.pose.pose.orientation.x = float(i[5])
                        to_save.pose.pose.orientation.y = float(i[6])
                        to_save.pose.pose.orientation.z = float(i[7])
                        to_save.pose.pose.orientation.w = float(i[8])
                        self.known_location[name] = to_save
        else:
            rospy.logerr('Invalid Service Call')

        return True


    def image_callback(self, msg):
        self.current_img = msg


    def pose_callback(self, msg):
        self.heading = msg


    def feedback_callback(self, feedback):
        self.server.publish_feedback(control_robotFeedback(progress='In Progresss'))


    def active_callback(self):
        rospy.loginfo('Action is active')   


    def action_callback(self, goal_name):
        rospy.loginfo('Control Robot Action Server Started')
        try:
            goal_og = self.known_location[goal_name.name]
        except:
            rospy.logerr("Invalid name entered")
            return None
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with move_base server')
        goal = MoveBaseGoal()
        goal.target_pose.header = goal_og.header
        goal.target_pose.pose = goal_og.pose.pose
        client.send_goal(goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback)
        client.wait_for_result()
        result = 1
        self.server.set_succeeded(control_robotResult(status=result))
        rospy.loginfo("Control Robot Action Completed")  


    def patrol_action_callback(self, something):
        rospy.loginfo('Patrol Action Server Started')
        bag = rosbag.Bag(self.filepath+'/Bag/Extra_credit.bag', 'w')
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        rospy.loginfo('Made contact with move_base server')
        for key, value in enumerate(self.known_location):
            goal_og = self.known_location[value]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = goal_og.header.frame_id
            goal.target_pose.pose = goal_og.pose.pose
            client.send_goal(goal, active_cb=self.active_callback, feedback_cb=self.feedback_callback)
            client.wait_for_result()
            bag.write('extra_credit',self.current_img)
        result = 1
        self.patrol_server.set_succeeded(patrolResult(status=result))
        rospy.loginfo("Patrol Action Completed")
        bag.close()



if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    controller = Robot_controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")