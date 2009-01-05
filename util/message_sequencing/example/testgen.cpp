#include "ros/node.h"
#include "std_msgs/PointStamped.h"
#include "ros/time.h"


class TestGen : public ros::node
{

public:

  TestGen() : ros::node("test_gen")
  {
    advertise<std_msgs::PointStamped>("orig",10);
  }

  bool spin()
  {
    float count = 0.0;
    while (ok())
    {
      usleep(100000);

      std_msgs::PointStamped p1;
      p1.header.stamp = ros::Time::now();
      p1.point.x = count;
      p1.point.y = 0.0;
      p1.point.z = 0.0;

      publish("orig", p1);

      count = count + 1.0;
    }
    return true;
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  TestGen t;
  t.spin();
  ros::fini();
}
