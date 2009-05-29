#include "executive_trex_pr2/ros_reactor.h"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"

namespace executive_trex_pr2{ 	

  /**
   * ROS Reactors will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  ROSReactor::ROSReactor(const LabelStr& agentName, const TiXmlElement& configData)
    : DbCore(agentName, configData){
    service_server_ = node_handle_.advertiseService("execute_goals", &ROSReactor::executeGoals, this);
  }

  ROSReactor::~ROSReactor() {}

  bool ROSReactor::executeGoals(ExecuteGoals::Request &req, ExecuteGoals::Response &resp){
    ROS_ERROR("ros:execute_goals, Service call made");
    // Aquire a lock

    // if the database is valid, add the goals
    // getPlanDatabase();
    // Release the lock
    resp.ok = true;

    return true;
  }

}
