#include "lifelong_mapping/graph_server.h"

namespace lifelong_mapping {

GraphServer::GraphServer(const ros::NodeHandle& node_handle)
  : nh_(node_handle)
{
  db_.reset(new Database("database"));

  query_service_ = nh_.advertiseService("/graph_server/query", &GraphServer::queryDatabase, this);
  register_service_ = nh_.advertiseService("/graph_server/register_data_topic",
                                           &GraphServer::registerDataTopic, this);
}

bool GraphServer::queryDatabase(Query::Request &req, Query::Response &rsp)
{
  rsp.matches.reserve(req.topics.size());
  BOOST_FOREACH(const TopicInfo& topic_info, req.topics) {
    // Construct and run query
    DbQueryRaw query;
    query.topic = topic_info.name;
    query.data_type = topic_info.data_type;
    query.md5 = topic_info.md5;
    db_->select(query);

    // Copy records into response
    rsp.matches.push_back(TopicHistory());
    TopicHistory &history = rsp.matches.back();
    history.info = topic_info;
    BOOST_FOREACH(DbQueryRaw::Record record, query.records) {
      history.messages.push_back(RawMessage());
      history.messages.back().serialized_data.assign((char*)record.data, record.length);
    }
  }
  
  return true;
}

bool GraphServer::registerDataTopic(RegisterDataTopic::Request &req,
                                    RegisterDataTopic::Response &rsp)
{
  ROS_INFO("Received request to register topic %s", req.topic.c_str());
  
  // See if we're already subscribed to the topic
  BOOST_FOREACH(const ros::Subscriber& sub, data_subs_)
    if (sub.getTopic() == req.topic)
      return true;

  ROS_INFO("Adding subscriber for %s", req.topic.c_str());
  data_subs_.push_back( nh_.subscribe<DatabaseItem>(req.topic, 1,
                                                    boost::bind(&GraphServer::dataCallback,
                                                                this, _1, req.topic)) );
  return true;
}

void GraphServer::dataCallback(const DatabaseItemConstPtr& msg, const std::string& topic)
{
  ROS_INFO("Got data message on topic %s", topic.c_str());
}

} //namespace lifelong_mapping
