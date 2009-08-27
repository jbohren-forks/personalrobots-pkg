#include <ros/ros.h>

#include "Hanoi.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanoi");
  ros::NodeHandle nh;

  Hanoi hanoi(nh);

  ros::spin();
}
