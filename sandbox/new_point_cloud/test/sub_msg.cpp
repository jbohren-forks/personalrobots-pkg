#include <ros/ros.h>
#include "new_point_cloud/PointCloud32.h"

void callback(const new_point_cloud::PointCloud32ConstPtr& msg)
{
  ROS_INFO("Received point_cloud");
  for (size_t i = 0; i < msg->points.size(); ++i) {
    const new_point_cloud::Point32 &pt = msg->points[i];
    ROS_INFO("\tx = %f, y = %f, z = %f, r = %u, g = %u, b = %u, w = %u",
             pt.x, pt.y, pt.z, pt.r, pt.g, pt.b, pt.w);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_msg_test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cloud", 1, callback);
  ros::spin();

  return 0;
}
