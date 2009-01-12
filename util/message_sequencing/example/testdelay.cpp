#include "ros/node.h"
#include "std_msgs/PointStamped.h"
#include "ros/time.h"


int g_argc;
char** g_argv;

class TestDelay : public ros::node
{

public:

  ros::Duration delay;
  int num;
  std_msgs::PointStamped point;

  TestDelay() : ros::node("test_delay", ros::node::ANONYMOUS_NAME)
  {
    delay.fromSec(atof(g_argv[1]));
    num = atoi(g_argv[2]);

    subscribe("orig",point,&TestDelay::cb,1);
    advertise<std_msgs::PointStamped>("delay",10);
  }

  void cb()
  {
    delay.sleep();
    point.point.y = num;
    publish("delay", point);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  g_argc = argc;
  g_argv = argv;
  TestDelay t;
  t.spin();
  ros::fini();
}
