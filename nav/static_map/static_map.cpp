#include <unistd.h>
#include "ros/node.h"
#include "std_msgs/rpcMap.h"

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
    
    // now, we respond to the query
    rpc.response.cell_size = 0.1; // 10cm grid
    rpc.response.map.width = 600;
    rpc.response.map.height = 600;
    // we have a gzipped version of the map already
    // i will provide some mechanism for setting a const
    // pointer in the message class to point to an 
    // externally-allocated buffer
    rpc.response.map.use_data(my_data_buf);
    */
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  StaticMap sm;
  sm.spin();
  return 0;
}

