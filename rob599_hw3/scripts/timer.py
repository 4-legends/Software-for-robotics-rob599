#!/usr/bin/env python

import sys
import rospy
import math
import time
import numpy as np
from sensor_msgs.msg import LaserScan
#######################################
# Laser Scan: 
#   Header: Seq, Stamp, frame_id
#   Angle_min, Angle_max, Angle_Increment, Time_Increment
#   Scan time, range_min, range_max, ranges, intensities 
#######################################

class Timer:
    def __init__(self, argv):
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)
        self.laser_sub = rospy.Subscriber('/node_time_taken', LaserScan, self.time_callback)
        self.scan_pub = rospy.Publisher('/timer_start', LaserScan, queue_size= 1)
        self.counter = 0
        self.print = 0
        self.filepath = argv[0]
        self.filepath = self.filepath.replace('scripts/timer.py', 'config/')
        if rospy.get_param('node_num') == 1:
            self.node = "Timer node to Timer node"
        elif rospy.get_param('node_num') == 2:
            self.node = "Timer node to Median node to Timer node"
        elif rospy.get_param('node_num') == 3:
            self.node = "Timer node to Median node to Noiser node to Timer node"


    def time_callback(self, msg):
        print('Python: Time taken by {0} is {1} secs'.format(self.node, time.time() - self.counter))
        if self.print == 0:
            self.print  = 1
            with open(self.filepath+'time.txt', 'a') as self.csv_writer:
                self.csv_writer.write('\nPython: Time taken by {0} is {1} secs\n'.format(self.node, time.time() - self.counter))


    def laser_callback(self, msg):
        self.scan_pub.publish(msg)
        self.counter = time.time()


if __name__ == '__main__':
    rospy.init_node('timer', anonymous=True)
    if sys.argv != 0:
        timer = Timer(sys.argv)
    else:
        rospy.logerr('System arguments are not given')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")