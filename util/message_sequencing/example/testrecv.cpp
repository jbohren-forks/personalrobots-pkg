#include "ros/node.h"
#include "std_msgs/PointStamped.h"
#include "message_sequencing/time_sequencer.h"

class TestRecv : public ros::node
{

  message_sequencing::TimeSequencer<std_msgs::PointStamped> ts;

public:

  TestRecv() : ros::node("test_recv")
         , ts(this, "delay", 
              boost::bind(&TestRecv::sequencedCb, this, _1),
              boost::bind(&TestRecv::droppedCb, this, _1),
              ros::Duration().fromSec(0.2), 100, 10)
  {

  }

  void sequencedCb(const boost::shared_ptr<std_msgs::PointStamped>& message)
  {
    printf("Got a stamped point from %f with: %f %f %f (%f late)\n", message->header.stamp.toSec(), message->point.x, message->point.y, message->point.z, (ros::Time::now() - message->header.stamp).toSec());
  }

  void droppedCb(const boost::shared_ptr<std_msgs::PointStamped>& message)
  {
    printf("Dropped a stamped point from %f with: %f %f %f\n", message->header.stamp.toSec(), message->point.x, message->point.y, message->point.z);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv);
  TestRecv t;
  t.spin();
  ros::fini();
}
