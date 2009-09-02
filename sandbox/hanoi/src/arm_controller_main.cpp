#include <ros/ros.h>

#include "ArmController.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_controller");
  ros::NodeHandle nh;

  ArmController ac(nh);

  ros::spin();
}
