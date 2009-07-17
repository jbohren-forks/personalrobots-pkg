#include "lifelong_mapping/graph_client.h"
#include <std_msgs/Float32.h>

static const char TOPIC[] = "/test/topic";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "query_test");
  ros::NodeHandle n;
  lifelong_mapping::GraphClient client;

  lifelong_mapping::DbQuery<std_msgs::Float32> query;
  query.topic = TOPIC;
  client.select(query);

  printf("Queried for %s, results:\n", TOPIC);
  BOOST_FOREACH(std_msgs::Float32::Ptr &record, query.records)
    printf("\tData = %f\n", record->data);
  
  return 0;
}
