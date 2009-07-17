#include "lifelong_mapping/database.h"

#include <std_msgs/Float32.h>
#include <boost/foreach.hpp>
#include <cstdio>

static const char TOPIC[] = "/test/topic";

using namespace lifelong_mapping;

int main(int argc, char** argv)
{
  std_msgs::Float32 msg;
  msg.data = 42.0;

  Database db("database");
  uint32_t id = db.insert(msg, TOPIC, 6);

  std_msgs::Float32 msg2;
  std::string topic;
  uint32_t node_id;
  db.get(msg2, id, topic, node_id);
  printf("ID = %u, topic = %s, node ID = %u, Data = %f\n", id, topic.c_str(), node_id, msg2.data);

  DbQuery<std_msgs::Float32> query;
  query.topic = TOPIC;
  db.select(query);
  printf("Queried for %s, results:\n", TOPIC);
  BOOST_FOREACH(std_msgs::Float32::Ptr &record, query.records)
    printf("\tData = %f\n", record->data);

  return 0;
}
