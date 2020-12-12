#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>

class Laser_Filter 
{
  public:
    ros::NodeHandle n;
    ros::Publisher filtered_laser_pub = n.advertise<sensor_msgs::LaserScan>("/laser_scan", 1000);
    void laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
};


void Laser_Filter::laser_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
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

  int median_filter_size;
  n.getParam("median_filter_size", median_filter_size);
  if (median_filter_size < 1)
  {
    median_filter_size = 1;
  }
  else if(median_filter_size > filtered_value_len/2 - 1)
  {
    median_filter_size = filtered_value_len/2 - 1;
  }
  int sizee = 2*median_filter_size+1;
  for(int i=0; i<filtered_value_len - median_filter_size -1; i++)
  {
    if(i<median_filter_size)
    {
      float not_a_median;
      filtered_values_ranges[i] = not_a_median;
    }
    else{
         std::vector<float> median_array(sizee, 0);
         int count = 0;
        for(int j= i - median_filter_size; j<sizee; j++)
        {
         median_array[count] = filtered_scan.ranges[j];
         count++;
        }
        std::sort(median_array.begin(), median_array.end());
        int median_idx = sizee/2;
        filtered_values_ranges[i] = median_array[median_idx];
        }
  }
  for(int i =0; i <filtered_value_len; i++)
  {
  filtered_scan.ranges[i] = filtered_values_ranges[i];
  }
  filtered_laser_pub.publish(filtered_scan);
  }


int main(int argc, char **argv)
{
  ros::init(argc, argv, "median_filter");

  ros::NodeHandle n;

  Laser_Filter laser;
  ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/base_scan", 1000, &Laser_Filter::laser_Callback, &laser);

  ros::spin();

  return 0;
}