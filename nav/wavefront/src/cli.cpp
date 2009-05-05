#include <cstdio>
#include <stdlib.h>
#include "ros/node.h"
#include "ros/publisher.h"
#include "nav_robot_actions/MoveBaseState.h"
#include "robot_msgs/PoseStamped.h"
#include "tf/tf.h"

class WavefrontCLI : public ros::Node
{
public:
  nav_robot_actions::MoveBaseState wf_state;
  robot_msgs::PoseStamped wf_goal;
  enum { WF_IDLE, WF_SEEKING_GOAL, WF_DONE } state;

  WavefrontCLI(double x, double y, double th)
  : ros::Node("wavefront_cli"), state(WF_IDLE)
  {
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::Quaternion(th, 0.0, 0.0), tf::Point(x, y, 0.0)), ros::Time::now(), "map");
    tf::PoseStampedTFToMsg(p, wf_goal);

    subscribe("state", wf_state, &WavefrontCLI::state_cb, 1);
    advertise("goal", wf_goal, &WavefrontCLI::goal_subscriber_callback, 1);
  }
  void state_cb()
  {
    if (state == WF_IDLE && wf_state.status.value == wf_state.status.ACTIVE)
      state = WF_SEEKING_GOAL;
    else if (state == WF_SEEKING_GOAL && wf_state.status.value != wf_state.status.ACTIVE)
      state = WF_DONE;
  }
  void deactivate_goal()
  {
    // Not sure what the right thing to do here is.
    //wf_goal.enable = 0;
    publish("goal", wf_goal);
    ros::Duration().fromSec(0.5).sleep(); // hack to try and wait for the message to go
  }

#if ROS_VERSION_MINIMUM(0, 5, 0)
  void goal_subscriber_callback(const ros::SingleSubscriberPublisher& pub)
#else
  void goal_subscriber_callback(const ros::PublisherPtr& pub)
#endif
  {
    publish("goal", wf_goal);
  }
};

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    printf("usage: ./cli X Y THETA TIMEOUT\n"
           "       where X and Y are in meters,\n"
           "       THETA is in radians,\n"
           "       and TIMEOUT is in seconds.\n");
    return 1;
  }
  ros::init(argc, argv);
  double max_secs = atof(argv[4]);
  WavefrontCLI wf_cli(atof(argv[1]), atof(argv[2]), atof(argv[3]));
  ros::Time t_start(ros::Time::now());
  while (wf_cli.ok() && wf_cli.state != WavefrontCLI::WF_DONE &&
         ros::Time::now() - t_start < ros::Duration().fromSec(max_secs))
    ros::Duration().fromSec(0.1).sleep();
  if (wf_cli.ok() && wf_cli.state != WavefrontCLI::WF_DONE) // didn't get there
  {
    printf("timeout\n");
    wf_cli.deactivate_goal();
  }
  else
    printf("success\n");

  return 0;
}
