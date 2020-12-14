#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>
#include <fstream>
#include <string>
#include <ros/package.h>

class Timer 
{
  public:
    double secs;
    int complete;
    double average_time;
    std::string filepath;
    int print = 0;
    ros::NodeHandle n;
    ros::Publisher timer_pub = n.advertise<sensor_msgs::LaserScan>("/timer_start", 1000);
    void laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void time_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};


void Timer::laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan filtered_scan;
  filtered_scan.header = msg->header;
  filtered_scan.angle_max = msg->angle_max;
  filtered_scan.angle_min = msg->angle_min;
  filtered_scan.ranges = msg->ranges;
  filtered_scan.angle_increment = msg->angle_increment;
  filtered_scan.time_increment = msg->time_increment;
  filtered_scan.scan_time = msg->scan_time;
  filtered_scan.intensities = msg->intensities;
  filtered_scan.range_min = msg->range_min;
  filtered_scan.range_max = msg->range_max;
  filtered_scan.ranges = msg -> ranges;
  timer_pub.publish(filtered_scan);
  if (complete ==0){
  complete = 1;
  secs =ros::Time::now().toSec();
  }
  }

void Timer::time_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  complete = 0;
  average_time = average_time + (ros::Time::now().toSec() - secs);
  print = print + 1;
  ROS_INFO("C++: Time taken by nodes is %f", (ros::Time::now().toSec() - secs));
  if(print == 100)
  {
    ROS_INFO("C++: Time taken by nodes for 100 steps is %f", average_time/100);
    std::ofstream outfile;
    outfile.open(filepath, std::ios_base::app);
    outfile << "\nC++: Time taken by nodes for 100 ste is " << (average_time/100) << "\n";
  }
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "timer");
  ros::NodeHandle n;
  std::string str1 ("rob599_hw3");
  std::string str2 ("rob599_hw3/config/time.txt");
  std::string path = ros::package::getPath("rob599_hw3");
  Timer timer;
  path.replace(path.find(str1), str1.length(), str2);
  timer.filepath = path;
  timer.complete = 0;
  timer.average_time = 0;
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &Timer::laser_Callback, &timer);
  ros::Subscriber sub1 = n.subscribe<sensor_msgs::LaserScan>("/node_time_taken", 1000, &Timer::time_Callback, &timer);
  ros::spin();
  return 0;
}
