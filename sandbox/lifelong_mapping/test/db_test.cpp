#include "lifelong_mapping/database.h"

#include <std_msgs/Float32.h>
#include <cstdio>

int main(int argc, char** argv)
{
  std_msgs::Float32 msg;
  msg.data = 42.0;

  lifelong_mapping::Database db("database");
  uint32_t id = db.insert(msg, "/test/topic", 6);

  std_msgs::Float32 msg2;
  std::string topic;
  uint32_t node_id;
  db.get(msg2, id, topic, node_id);

  printf("ID = %u, topic = %s, node ID = %u, Data = %f\n", id, topic.c_str(), node_id, msg2.data);

  return 0;
}
