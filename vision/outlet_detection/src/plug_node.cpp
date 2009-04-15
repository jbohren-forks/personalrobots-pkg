#include "outlet_detection/plug_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ros::Node node("plug_tracker");
  PlugTracker tracker(node);
  
  node.spin();

  return 0;
}
