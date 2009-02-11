#include "ros/node.h"
#include "robot_msgs/PointStamped.h"
#include "ros/time.h"


class TestGen : public ros::Node
{

public:

  TestGen() : ros::Node("test_gen")
  {
    advertise<robot_msgs::PointStamped>("orig",10);
  }

  bool spin()
  {
    float count = 0.0;
    while (ok())
    {
      usleep(100000);

      robot_msgs::PointStamped p1;
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
  
}
