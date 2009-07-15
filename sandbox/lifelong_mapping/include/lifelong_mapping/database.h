#ifndef LIFELONG_MAPPING_DATABASE_H
#define LIFELONG_MAPPING_DATABASE_H

#include <db_cxx.h>
#include <boost/scoped_ptr.hpp>
#include <ros/message.h>

namespace lifelong_mapping {

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
