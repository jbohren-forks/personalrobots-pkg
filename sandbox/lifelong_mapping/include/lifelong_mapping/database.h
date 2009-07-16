#ifndef LIFELONG_MAPPING_DATABASE_H
#define LIFELONG_MAPPING_DATABASE_H

#include <db_cxx.h>
#include <boost/scoped_ptr.hpp>
#include <ros/message.h>

namespace lifelong_mapping {

struct DbQueryRaw
{
  std::string topic;
  std::string md5;
  std::string data_type;
  std::vector<uint32_t> node_ids;

  std::vector<uint8_t*> records;
};

template <typename M>
struct DbQuery
{
  std::string topic;
  std::vector<uint32_t> node_ids;

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
  
private:
  typedef boost::scoped_ptr<Db> DbPtr;
  
  DbEnv env_;
  DbPtr db_;
  DbPtr sequence_db_;
  boost::scoped_ptr<DbSequence> sequence_;
  DbPtr topic_index_;
  DbPtr node_index_;
};

} //namespace lifelong_mapping

#endif
