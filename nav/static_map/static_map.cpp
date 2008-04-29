#include <unistd.h>
#include "ros/node.h"

using namespace ros;

class StaticMap : public node
{
public:
  StaticMap() : node("static_map")
  {
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StaticMap sm;
  sm.spin();
  return 0;
}

