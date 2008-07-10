#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// roscpp
#include <ros/node.h>

// Provides a blocking navigation service for higher-level sequencing
#include "wavefront_player/NavigateToPoint.h"

#define USAGE "send_goal <x> <y> <a> [stuck_time]"

class SendGoalNode : public ros::node
{
  public:
    std_msgs::Planner2DGoal goalMsg;

    SendGoalNode() : ros::node("send_goal")
    {
      advertise<std_msgs::Planner2DGoal>("goal");
    }

    void sendGoal(double x, double y, double a, ros::Duration stuck_time)
    {
      wavefront_player::NavigateToPoint::request req;
      req.goal.goal.x = x;
      req.goal.goal.y = y;
      req.goal.goal.th = a;
      req.goal.enable = true;
      req.stuck_time = stuck_time;
      wavefront_player::NavigateToPoint::response rep;

      if(!ros::service::call("NavigateToPoint", req, rep))
        puts("plan service failed");
      if(rep.result == "failure")
        puts("planning failed");
      else
        puts("planning succeeded; robot is on the move");
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

  ros::Duration stuck_time;
  if(argc > 4)
    stuck_time = ros::Duration(atof(argv[4]));
  else
    stuck_time = ros::Duration(1.0);


  SendGoalNode n;

  usleep(1000000);
  n.sendGoal(atof(argv[1]),atof(argv[2]),atof(argv[3])*M_PI/180.0,stuck_time);

  //n.spin();

  ros::fini();
  
  return(0);
}
