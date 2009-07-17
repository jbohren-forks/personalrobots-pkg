#ifndef LIFELONG_MAPPING_GRAPH_SERVER_H
#define LIFELONG_MAPPING_GRAPH_SERVER_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "lifelong_mapping/database.h"
#include "lifelong_mapping/Query.h"

namespace lifelong_mapping {

class GraphServer
{
public:
  GraphServer(const ros::NodeHandle& node_handle);

  bool queryDatabase(lifelong_mapping::Query::Request &req,
                     lifelong_mapping::Query::Response &rsp);


private:
  ros::NodeHandle nh_;
  ros::ServiceServer query_service_;
  
  boost::scoped_ptr<Database> db_;
};

} //namespace lifelong_mapping

#endif
