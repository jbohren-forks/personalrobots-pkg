#include <cstdio>
#include <stdlib.h>
#include "ros/node.h"
#include "ros/publisher.h"
#include "std_msgs/Planner2DState.h"
#include "std_msgs/Planner2DGoal.h"

class WavefrontCLI : public ros::node
{
public:
  std_msgs::Planner2DState wf_state;
  std_msgs::Planner2DGoal wf_goal;
  enum { WF_IDLE, WF_SEEKING_GOAL, WF_DONE } state;

  WavefrontCLI(double x, double y, double th)
  : ros::node("wavefront_cli"), state(WF_IDLE)
  {
    wf_goal.goal.x  = x;
    wf_goal.goal.y  = y;
    wf_goal.goal.th = th;
    wf_goal.enable = 1;
    subscribe("state", wf_state, &WavefrontCLI::state_cb, 1);
    advertise("goal", wf_state, &WavefrontCLI::goal_subscriber_callback, 1);
  }
  void state_cb()
  {
    if (state == WF_IDLE && wf_state.active)
      state = WF_SEEKING_GOAL;
    else if (state == WF_SEEKING_GOAL && !wf_state.active)
      state = WF_DONE;
  }
  void deactivate_goal()
  {
    wf_goal.enable = 0;
    publish("goal", wf_goal);
    ros::Duration().fromSec(0.5).sleep(); // hack to try and wait for the message to go
  }
  void goal_subscriber_callback(const ros::PublisherPtr& pub)
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
  ros::fini();
  return 0;
}
