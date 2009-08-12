#include "outlet_detection/outlet_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "outlet_detector");
  ros::NodeHandle node;
  OutletTracker tracker(node);
  
  ros::spin();

  return 0;
}
