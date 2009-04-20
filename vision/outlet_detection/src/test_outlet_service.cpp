#include <ros/node.h>
#include <outlet_detection/OutletDetection.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node n("outlet_service_test");

  outlet_detection::OutletDetection::Request req;
  outlet_detection::OutletDetection::Response res;

  req.point.header.frame_id = "high_def_frame";
  req.point.point.x = 0.941047;
  req.point.point.y = -0.013503;
  req.point.point.z = -0.097568;
  
  if (ros::service::call("/outlet_detector/fine_outlet_detect", req, res)) {
    printf("Outlet found\n");
  }
  else {
    printf("Outlet not found\n");
  }
}
