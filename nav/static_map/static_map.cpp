#include <unistd.h>
#include "ros/node.h"
#include "std_rpc/map.h"

using namespace ros;

class StaticMap : public node
{
public:
  StaticMap() : node("static_map")
  {
    advertise("map", &StaticMap::map_cb);
  }
  void map_cb(rpcMap &rpc)
  {
    // this code must be re-entrant, or else you must
    // mutex it yourself
    /*
    the extents of the query could be accessed this way:
    rpc.request.extents.x0
    rpc.request.extents.y0
    rpc.request.extents.x1
    rpc.request.extents.y1
    */
    rpc.response.cell_size = 0.1; // 10cm grid
    rpc.response.map.width = 600;
    rpc.response.map.height = 600;
    // copy over the buffer
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StaticMap sm;
  sm.spin();
  return 0;
}

