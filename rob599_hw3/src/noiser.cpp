#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>

class Noiser 
{
  public:
    ros::NodeHandle n;
    ros::Publisher filtered_laser_pub = n.advertise<sensor_msgs::LaserScan>("/gaus_err_laser_scan", 1000);
    void laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};


void Noiser::laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int filtered_value_len = sizeof(msg ->ranges)/sizeof(msg ->ranges[0]);
  std::vector<float> filtered_values_ranges(filtered_value_len, 0);
  
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

  double laser_noise_variance;
  n.getParam("laser_noise_variance", laser_noise_variance);
  if (laser_noise_variance <= 0 )
  {
    laser_noise_variance = 0.1;
  }
  const double mean = 0.0;
  const double stddev = laser_noise_variance;
  std::default_random_engine generator;
  std::normal_distribution<double> dist(mean, stddev);
  for(int i =0; i <filtered_value_len; i++)
  {
  filtered_values_ranges[i] = filtered_scan.ranges[i] + dist(generator);
  }  

  for(int i =0; i <filtered_value_len; i++)
  {
  filtered_scan.ranges[i] = filtered_values_ranges[i];
  }
  filtered_laser_pub.publish(filtered_scan);
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "noiser");

  ros::NodeHandle n;

  Noiser noiser;
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &Noiser::laser_Callback, &noiser);

  ros::spin();

  return 0;
}