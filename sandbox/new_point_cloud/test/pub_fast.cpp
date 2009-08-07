#include <ros/ros.h>
#include "new_point_cloud/fast_point_cloud_msg.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_fast_test");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<new_point_cloud::FastPointCloud>("cloud", 1);

  new_point_cloud::FastPointCloud msg;
  msg.point_cloud.allocate(2, 3);
  for (size_t i = 0; i < 2*3; ++i) {
    new_point_cloud::Point &pt = msg.point_cloud.points[i];
    pt.x = pt.y = pt.z = (float)i;
    pt.color.r = pt.color.g = pt.color.b = pt.color.w = i;
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
