#ifndef H_ExecDefs
#define H_ExecDefs

#include "Id.hh"
#include "ros/node.h"
#include "std_msgs/Planner2DState.h"
#include "std_msgs/Planner2DGoal.h"
//#include "std_msgs/MsgToken.h"


using namespace EUROPA;
using namespace std;

namespace TREX {

  enum PlannerState {
    UNDEFINED = 0,
    ACTIVE,
    INACTIVE
  };

  class Executive;
  typedef Id<Executive> ExecutiveId;
  class Monitor;
  typedef Id<Monitor> MonitorId;
  class GoalManager;

  std::string toString(const std_msgs::Planner2DState& s);
  std::string toString(const std_msgs::Planner2DGoal& g);

  void initROSExecutive(bool playback);
}

using namespace TREX;
#endif
