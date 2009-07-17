#include "lifelong_mapping/graph_server.h"
#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "graph_server");
  ros::NodeHandle n;
  lifelong_mapping::GraphServer server(n);

  ros::spin();

  return 0;
}
