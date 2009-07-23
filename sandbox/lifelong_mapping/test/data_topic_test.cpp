#include "lifelong_mapping/graph_client.h"
#include <std_msgs/Float32.h>

static const char TOPIC[] = "/test/topic";

using namespace lifelong_mapping;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_topic_test");
  ros::NodeHandle n;
  GraphClient client(n);

  DatabasePublisher data_pub = client.advertiseData<std_msgs::Float32>(TOPIC, 1);
  //ros::Publisher data_pub = n.advertise<std_msgs::Float32>(TOPIC, 1);
  ros::Rate loop_rate(4);
  int count = 0;
  std_msgs::Float32 f;
  while (n.ok()) {
    f.data = 42.0f;
    data_pub.publish(f, count);
    //data_pub.publish(f);
#if 0
    {
      std_msgs::Float32Ptr f_ptr( new std_msgs::Float32(f) );
      data_pub.publish(f_ptr);
    }
#endif
    ROS_INFO("Published data message");
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
