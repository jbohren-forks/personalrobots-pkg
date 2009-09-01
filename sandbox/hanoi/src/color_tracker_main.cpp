#include <ros/ros.h>

#include "ColorTracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanoi");
  ros::NodeHandle nh;

  ColorTracker ct(nh);

  ros::spin();
}
