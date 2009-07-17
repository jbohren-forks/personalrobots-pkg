#include "lifelong_mapping/graph_server.h"

namespace lifelong_mapping {

GraphServer::GraphServer(const ros::NodeHandle& node_handle)
  : nh_(node_handle)
{
  db_.reset(new Database("database"));

  query_service_ = nh_.advertiseService("/graph_server/query", &GraphServer::queryDatabase, this);
}

bool GraphServer::queryDatabase(lifelong_mapping::Query::Request &req,
                                lifelong_mapping::Query::Response &rsp)
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

} //namespace lifelong_mapping
