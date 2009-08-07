#include <ros/ros.h>
#include "new_point_cloud/PointCloud32.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_msg_test");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<new_point_cloud::PointCloud32>("cloud", 1);

  new_point_cloud::PointCloud32 msg;
  msg.width = 2;
  msg.height = 3;
  msg.points.resize(msg.width*msg.height);
  for (size_t i = 0; i < msg.points.size(); ++i) {
    new_point_cloud::Point32 &pt = msg.points[i];
    pt.x = pt.y = pt.z = (float)i;
    pt.r = pt.g = pt.b = pt.w = i;
  }
  
  ros::Rate loop_rate(1);
  while (n.ok())
  {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
