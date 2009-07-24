#ifndef LIFELONG_MAPPING_GRAPH_SERVER_H
#define LIFELONG_MAPPING_GRAPH_SERVER_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "lifelong_mapping/database.h"
#include "lifelong_mapping/pose_graph.h"
#include "lifelong_mapping/Query.h"
#include "lifelong_mapping/RegisterDataTopic.h"
#include "lifelong_mapping/DatabaseItem.h"
#include "lifelong_mapping/Constraint.h"

namespace lifelong_mapping {

class GraphServer
{
public:
  GraphServer(const ros::NodeHandle& node_handle);

  bool queryDatabase(Query::Request &req, Query::Response &rsp);

  bool registerDataTopic(RegisterDataTopic::Request &req, RegisterDataTopic::Response &rsp);

  // @todo: rename and/or make private
  void dataCallback(const DatabaseItemConstPtr& msg, const std::string& topic);

  void constraintCallback(const ConstraintConstPtr& msg);
  
private:
  ros::NodeHandle nh_;
  ros::ServiceServer query_service_;
  ros::ServiceServer register_service_;
  ros::V_Subscriber data_subs_;
  ros::Subscriber constraint_sub_;

  PoseGraph graph_;
  boost::scoped_ptr<Database> db_;
};

} //namespace lifelong_mapping

#endif
