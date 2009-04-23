#include "outlet_detection/outlet_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node node("outlet_detector");
  OutletTracker tracker(node);
  
  node.spin();

  return 0;
}
