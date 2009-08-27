#include <ros/ros.h>

#include "Hanoi.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hanoi");
  Hanoi hanoi;

  ros::spin();
}
