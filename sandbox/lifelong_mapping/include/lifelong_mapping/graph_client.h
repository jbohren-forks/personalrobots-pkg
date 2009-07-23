#ifndef LIFELONG_MAPPING_GRAPH_CONNECTION_H
#define LIFELONG_MAPPING_GRAPH_CONNECTION_H

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include "lifelong_mapping/Query.h"
#include "lifelong_mapping/RegisterDataTopic.h"
#include "lifelong_mapping/database_query.h"
#include "lifelong_mapping/database_publisher.h"

namespace lifelong_mapping {

class GraphClient
{
public:
  // @todo: take ROS namespace of graph server?
  GraphClient(const ros::NodeHandle& node_handle);

  template <typename M>
  bool select(DbQuery<M> &query);

  // @todo: other advertise versions?
  template <typename M>
  DatabasePublisher advertiseData(const std::string& topic, uint32_t queue_size);

  // @todo: uint32_t addNode();
  // @todo: constraint broadcaster? addConstraint()?
  
private:
  ros::NodeHandle nh_;
  //ros::ServiceClient query_service_;
};


template <typename M> bool GraphClient::select(DbQuery<M> &query)
{
  query.records.clear();
  
  Query::Request req;
  Query::Response rsp;

  TopicInfo topic;
  topic.name = query.topic;
  topic.data_type = M::__s_getDataType();
  topic.md5 = M::__s_getMD5Sum();
  req.topics.push_back(topic);
  req.node_ids = query.node_ids;

  // @todo: use ServiceClient instead of ros::service::call
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

template <typename M>
DatabasePublisher GraphClient::advertiseData(const std::string& topic, uint32_t queue_size)
{
  // Register data topic with the graph server
  // @todo: use ServiceClient
  RegisterDataTopic::Request req;
  RegisterDataTopic::Response rsp;
  req.topic = topic;
  if (!ros::service::call("/graph_server/register_data_topic", req, rsp)) {
    ROS_ERROR("Call to register_data_topic service failed!");
  }

  // Create publisher for the data topic
  typedef DatabaseItemFactoryT<M> Factory;
  DatabasePublisher dp;
  dp.factory_.reset( new Factory );
  dp.publisher_ = nh_.advertise<typename Factory::ItemType>(topic, queue_size);
  return dp;
}

} //namespace lifelong_mapping

#endif
