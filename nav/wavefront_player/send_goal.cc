#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// roscpp
#include <ros/node.h>

// The messages that we'll use
#include <std_msgs/MsgPlanner2DGoal.h>

#define USAGE "send_goal <x> <y> <a>"

class SendGoalNode : public ros::node
{
  public:
    MsgPlanner2DGoal goalMsg;

    SendGoalNode() : ros::node("send_goal")
    {
      advertise<MsgPlanner2DGoal>("goal");
    }

    void sendGoal(double x, double y, double a)
    {
      goalMsg.goal.x = x;
      goalMsg.goal.y = y;
      goalMsg.goal.th = a;
      goalMsg.enable = true;
      publish("goal", goalMsg);
    }
};

int
main(int argc, char** argv)
{
  if(argc < 4)
  {
    puts(USAGE);
    exit(-1);
  }
  ros::init(argc,argv);

  SendGoalNode n;

  n.sendGoal(atof(argv[1]),atof(argv[2]),atof(argv[3])*M_PI/180.0);

  n.spin();
  
  return(0);
}
