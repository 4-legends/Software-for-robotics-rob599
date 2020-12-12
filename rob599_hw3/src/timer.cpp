#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>
#include <fstream>
#include <string>


class Timer 
{
  public:
    double secs;
    //std::string filepath;
    //int print = 0;
    ros::NodeHandle n;
    ros::Publisher timer_pub = n.advertise<sensor_msgs::LaserScan>("/timer_start", 1000);
    void laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void time_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
    std::cout << 'Starts class' << std::endl;   
};


void Timer::laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO('Im Here')
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
  secs =ros::Time::now().toSec();
  }

void Timer::time_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO('C++: Time taken by Timer node to Timer node is %f', (ros::Time::now().toSec() - secs));
  /*if(print == 0)
  {
    print = 1;
    std::ofstream outfile;
    outfile.open(filepath, std::ios_base::app);
    outfile << '\nC++: Time taken by Timer node to Timer node is %f\n', (ros::Time::now().toSec() - secs);
  }*/
  }

int main(int argc, char **argv)
{
  std::cout << 'Starts' << std::endl;
  ros::init(argc, argv, "timer");
  //ROS_INFO('Right here')
  ros::NodeHandle n;
  //std::string str1 ('scripts/timer.py');
  //std::string str2 ('/rob599_hw3');
  Timer timer;
  /*timer.filepath = argv[0]
  if (timer.filepath.find(str1)!=std::string::npos)
  {
    timer.filepath.replace(timer.filepath.find(str1), str1.length(),"config/time.txt")
  }
  else if (timer.filepath.find('config')!=std::string::npos)
  {
   timer.filepath.replace(timer.filepath.find(str2), str2.length(),"/rob599_hw3/config/time.txt") 
  }*/
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &Timer::laser_Callback, &timer);
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/node_time_taken", 1000, &Timer::time_Callback, &timer);
  //ROS_INFO('Did this')
  ros::spin();

  return 0;
}