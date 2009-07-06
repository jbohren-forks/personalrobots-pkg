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

  void insert(int node_id, const ros::Message& msg);

  bool get(int node_id, ros::Message& msg);
  
private:
  DbEnv env_;
  boost::scoped_ptr<Db> db_;
};

} //namespace lifelong_mapping

#endif
