#include <ros/ros.h>

#include "color_gui.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cmvision");
  ros::NodeHandle nh;

  ColorGui *cg = new ColorGui(&nh);

  cg->Run();

  delete cg;

  return 0;
}
