#include "lifelong_mapping/graph_client.h"

namespace lifelong_mapping {

GraphClient::GraphClient(const ros::NodeHandle& node_handle)
  : nh_(node_handle)
{
}

ros::Publisher GraphClient::advertiseConstraint(uint32_t queue_size)
{
  return nh_.advertise<Constraint>("/graph_server/constraint", queue_size);
}

} //namespace lifelong_mapping
