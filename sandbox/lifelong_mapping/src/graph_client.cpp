#include "lifelong_mapping/graph_client.h"

namespace lifelong_mapping {

GraphClient::GraphClient(const ros::NodeHandle& node_handle)
  : nh_(node_handle)
{
}

} //namespace lifelong_mapping
