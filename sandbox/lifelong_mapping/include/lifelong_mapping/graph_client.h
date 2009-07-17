#ifndef LIFELONG_MAPPING_GRAPH_CONNECTION_H
#define LIFELONG_MAPPING_GRAPH_CONNECTION_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "lifelong_mapping/Query.h"
#include "lifelong_mapping/database_query.h"

namespace lifelong_mapping {

class GraphClient
{
public:
  //GraphClient(); // take ROS namespace of graph server?

  template <typename M>
  bool select(DbQuery<M> &query);

private:
  //ros::ServiceClient query_service_;
};


template <typename M> bool GraphClient::select(DbQuery<M> &query)
{
  query.records.clear();
  
  lifelong_mapping::Query::Request req;
  lifelong_mapping::Query::Response rsp;

  lifelong_mapping::TopicInfo topic;
  topic.name = query.topic;
  topic.data_type = M::__s_getDataType();
  topic.md5 = M::__s_getMD5Sum();
  req.topics.push_back(topic);
  req.node_ids = query.node_ids;

  if (!ros::service::call("/graph_server/query", req, rsp)) {
    ROS_ERROR("Call to query service failed!");
    return false;
  }

  BOOST_FOREACH(const lifelong_mapping::TopicHistory& history, rsp.matches) {
    BOOST_FOREACH(const lifelong_mapping::RawMessage& raw, history.messages) {
      typename M::Ptr msg(new M);
      msg->deserialize((uint8_t*)&raw.serialized_data[0]);
      query.records.push_back(msg);
    }
  }

  return true;
}

} //namespace lifelong_mapping

#endif
