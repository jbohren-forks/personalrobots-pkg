#include "outlet_detection/plug_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plug_detector");
  ros::NodeHandle node;
  PlugTracker tracker(node);
  
  ros::spin();

  return 0;
}
