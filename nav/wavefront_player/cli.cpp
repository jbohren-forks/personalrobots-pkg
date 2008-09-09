#include <cstdio>
#include "ros/node.h"
#include "std_msgs/Planner2DState.h"
#include "std_msgs/Planner2DGoal.h"

class WavefrontCLI : public ros::node
{
public:
  std_msgs::Planner2DState wf_state;
  std_msgs::Planner2DGoal wf_goal;
  bool goal_sent;

  WavefrontCLI(double x, double y, double th, double timeout) 
  : ros::node("wavefront_cli"), goal_sent(false)
  {
    wf_goal.goal.x  = x;
    wf_goal.goal.y  = y;
    wf_goal.goal.th = th;
    wf_goal.enable = 1;
    subscribe("state", wf_state, &WavefrontCLI::state_cb, 1);
    advertise<std_msgs::Planner2DGoal>("goal", 1);
  }
  void state_cb()
  {
    printf("wavefront active: %d\n", wf_state.active);
    if (goal_sent && !wf_state.active) // todo, add better time delay
      raise(SIGINT); // bye bye
  }
  virtual void peer_subscribe(const std::string &topic_name, 
                              ros::pub_sub_conn * const psc)
  {
    if (topic_name == "goal")
    {
      publish("goal", wf_goal);
      goal_sent = true;
      sleep(1);
    }
  }
  /*
    printf("a peer subscribed on topic [%s].\n", topic_name.c_str());
    {
      printf("publishing: %f %f %f\n", wf_goal.goal.x, wf_goal.goal.y, wf_goal.goal.th);
      psc->publish(wf_goal);
    }
  }
  */
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
  WavefrontCLI wf_cli(atof(argv[1]), atof(argv[2]), 
                      atof(argv[3]), atof(argv[4]));
  wf_cli.spin();
  ros::fini();
  return 0;
}
