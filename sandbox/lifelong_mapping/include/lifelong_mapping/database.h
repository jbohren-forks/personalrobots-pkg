#ifndef LIFELONG_MAPPING_DATABASE_H
#define LIFELONG_MAPPING_DATABASE_H

#include <db_cxx.h>
#include <boost/scoped_ptr.hpp>
#include <ros/message.h>

namespace lifelong_mapping {

struct DbQueryRaw
{
  // Inputs
  std::string topic;
  std::string data_type;
  std::string md5;
  std::vector<uint32_t> node_ids;

  // Outputs
  std::vector<uint8_t*> records;

  template <typename M> void setMessageType();
};

template <typename M>
struct DbQuery
{
  // Inputs
  std::string topic;
  std::vector<uint32_t> node_ids;

  // Outputs
  std::vector<typename M::Ptr> records;
};

class Database
{
public:
  Database(const std::string& path);

  ~Database();

  uint32_t insert(const ros::Message& msg, const std::string& topic, uint32_t node_id);

  // FIXME: should these be private?
  void insert(const ros::Message& msg, const std::string& topic, uint32_t node_id, uint32_t item_id);
  uint32_t nextId();

  bool get(ros::Message& msg, uint32_t item_id, std::string& topic, uint32_t& node_id);

  // query
  template <typename M> bool query(DbQuery<M>& query);
  bool query(DbQueryRaw& query, DbTxn* parent_txn = NULL); // FIXME: don't expose DbTxn
  
private:
  typedef boost::scoped_ptr<Db> DbPtr;
  
  DbEnv env_;
  DbPtr db_;
  DbPtr sequence_db_;
  boost::scoped_ptr<DbSequence> sequence_;
  DbPtr topic_index_;
  DbPtr node_index_;
};


template <typename M>
void DbQueryRaw::setMessageType()
{
  data_type = M::__s_getDataType();
  md5 = M::__s_getMD5Sum();
}

template <typename M>
bool Database::query(DbQuery<M>& query)
{
  // Convert to raw data query
  DbQueryRaw raw_query;
  raw_query.topic = query.topic;
  raw_query.setMessageType<M>();
  raw_query.node_ids = query.node_ids;
  
  DbTxn *txn = NULL;
  env_.txn_begin(NULL, &txn, 0);
  if (!this->query(raw_query, txn))
    return false;

  // Deserialize into messages
  query.records.resize(raw_query.records.size());
  for (size_t i = 0; i < query.records.size(); ++i) {
    query.records[i].reset( new M );
    query.records[i]->deserialize(raw_query.records[i]);
  }

  txn->commit(0);
  return true;
}

} //namespace lifelong_mapping

#endif
