#include "outlet_detection/outlet_tracker.h"
//#include <boost/format.hpp>
//#include <unistd.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  //ros::Node node((boost::format("outlet_detector_%u") % getpid()).str());
  ros::Node node("outlet_detector");
  OutletTracker tracker(node);
  
  node.spin();

  return 0;
}
