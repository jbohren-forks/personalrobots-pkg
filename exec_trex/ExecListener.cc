/**
 * This class will go away when we have a logging service, if not sooner.
 * It is used to enable monitoring of states in the Executive.
 */

#include "ros/node.h"
#include "std_msgs/MsgToken.h"

class ExecListener : public ros::node
{
public:

  ExecListener() : ros::node("execListener"){
    subscribe("navigator", msg, &ExecListener::navigator_cb);
  }

  void navigator_cb(){
    printf("Monitor received status update: [%s]\n", msg.predicate.c_str());
  }

private:
  MsgToken msg;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  ExecListener l;
  l.spin();
  l.shutdown();
  return 0;
}
