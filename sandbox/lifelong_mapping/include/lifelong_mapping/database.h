#ifndef LIFELONG_MAPPING_DATABASE_H
#define LIFELONG_MAPPING_DATABASE_H

#include <db_cxx.h>
#include <boost/scoped_ptr.hpp>
#include <ros/message.h>

#include "lifelong_mapping/database_query.h"

namespace lifelong_mapping {

class Database
{
public:
  Database(const std::string& path);

  ~Database();

  uint32_t insert(const ros::Message& msg, const std::string& topic, uint32_t node_id);
  //uint32_t insert(raw data); // or as RawMessage?

  // FIXME: should these be private?
  void insert(const ros::Message& msg, const std::string& topic, uint32_t node_id, uint32_t item_id);
  //uint32_t insert(raw data, item_id);
  uint32_t nextId();

  bool get(ros::Message& msg, uint32_t item_id, std::string& topic, uint32_t& node_id);

  // query
  template <typename M> bool select(DbQuery<M>& query);
  bool select(DbQueryRaw& query, DbTxn* parent_txn = NULL); // FIXME: don't expose DbTxn?
  
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
bool Database::select(DbQuery<M>& query)
{
  DbQueryRaw raw_query = query.asRawQuery();
  
  DbTxn *txn = NULL;
  env_.txn_begin(NULL, &txn, 0);
  if (!select(raw_query, txn))
    return false;

  query.deserializeRecords(raw_query);
  
  txn->commit(0);
  return true;
}

} //namespace lifelong_mapping

#endif
