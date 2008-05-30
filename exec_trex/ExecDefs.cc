#include "ExecDefs.hh"

namespace TREX {

  std::string toString(const MsgPlanner2DState& s){
    std::stringstream ss;
    ss << "At(" << s.pos.x << ", " << s.pos.y << ") Going(" << s.goal.x << ", " << s.goal.y << ") Planner is:" << (s.done == 1 ? "INACTIVE" : "ACTIVE");
    return ss.str();
  }

  std::string toString(const MsgPlanner2DGoal& g){
    std::stringstream ss;
    ss << "Going(" << g.goal.x << ", " << g.goal.y << ")";
    return ss.str();
  }

}
