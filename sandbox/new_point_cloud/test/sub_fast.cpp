#include <ros/ros.h>
#include "new_point_cloud/fast_point_cloud_msg.h"

void callback(const new_point_cloud::FastPointCloudConstPtr& msg)
{
  ROS_INFO("Received point_cloud");
  size_t num_pts = msg->point_cloud.width * msg->point_cloud.height;
  for (size_t i = 0; i < num_pts; ++i) {
    const new_point_cloud::Point &pt = msg->point_cloud.points[i];
    ROS_INFO("\tx = %f, y = %f, z = %f, r = %u, g = %u, b = %u, w = %u",
             pt.x, pt.y, pt.z, pt.color.r, pt.color.g, pt.color.b, pt.color.w);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_fast_test");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("cloud", 1, callback);
  ros::spin();

  return 0;
}
