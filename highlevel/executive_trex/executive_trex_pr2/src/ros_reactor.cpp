#include "executive_trex_pr2/ros_reactor.h"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"
#include "Token.hh"
#include "Domains.hh"
#include "ConstrainedVariable.hh"

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
    ROS_INFO("ros:execute_goals, Service call made");
    TREX_INFO("ros:execute_goals", "Service call made");

    lock_.lock();

    // Get the client to work with
    DbClientId client = getAssembly().getPlanDatabase()->getClient();
    resp.ok = true;

    for(std::vector<int32_t>::const_iterator it = req.outlet_ids.begin(); it != req.outlet_ids.end(); ++it){
      // Allocate a token - it should be inactive but not rejectable - cannot deny the truth
      TokenId token = client->createToken("M2Goals.Active", true);
      ConstrainedVariableId outlet_id_param = token->getVariable("outlet_id", false);
      int32_t outlet_id = *it;
      outlet_id_param->restrictBaseDomain(IntervalIntDomain(outlet_id, outlet_id));
      if(getAssembly().getConstraintEngine()->propagate()){
	getGoals().insert(token);
      }
      else {
	ROS_ERROR("Failed to add goal for outlet %d. Skipping.", outlet_id);
	token->discard();
	resp.ok = false;
      }
    }

    lock_.unlock();
    return true;

  }

  void ROSReactor::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    lock_.lock();
    DbCore::handleInit(initialTick, serversByTimeline, observer);
    lock_.unlock();
  }

  void ROSReactor::handleTickStart(){
    lock_.lock();
    DbCore::handleTickStart();
    lock_.unlock();
  }

  bool ROSReactor::synchronize(){
    bool result(false);
    lock_.lock();
    result = DbCore::synchronize();
    lock_.unlock();
    return result;
  }

  bool ROSReactor::hasWork(){
    bool result(false);
    lock_.lock();
    result = DbCore::hasWork();
    lock_.unlock();
    return result;
  }

  void ROSReactor::resume(){
    lock_.lock();
    DbCore::resume();
    lock_.unlock();
  }

}
