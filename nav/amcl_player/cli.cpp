#include <cstdio>
#include "ros/node.h"
#include "std_msgs/RobotBase2DOdom.h"

class AmclCLI: public ros::node
{
public:
  std_msgs::RobotBase2DOdom pose_msg;
  bool done;

  AmclCLI(): ros::node("amcl_cli"), done(false)
  {
    subscribe("localizedpose", pose_msg, &AmclCLI::pose_cb, 1);
  }
  void pose_cb()
  {
    if (done)
      return;
    time_t t = time(NULL);
    struct tm *tms = localtime(&t);
    printf("%d %02d %02d %02d %02d %02d %.3f %.3f %.3f\n", 
           tms->tm_year + 1900, tms->tm_mon + 1, tms->tm_mday,
           tms->tm_hour, tms->tm_min, tms->tm_sec,
           pose_msg.pos.x, pose_msg.pos.y, pose_msg.pos.th);
    done = true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  AmclCLI amcl_cli;
  ros::Time t_start(ros::Time::now());
  while (amcl_cli.ok() && ros::Time::now() - t_start < ros::Duration().fromSec(2.0) && !amcl_cli.done)
    ros::Duration().fromSec(0.1).sleep();
  if (!amcl_cli.done)
    printf("failed\n");
  ros::fini();
  return 0;
}

