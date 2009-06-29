#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"
#include "Token.hh"
#include "Domains.hh"
#include "ConstrainedVariable.hh"

#include "trex_pr2/master_reactor.h"

using namespace TREX;
using namespace EUROPA;

namespace trex_pr2{ 	

  /**
   * ROS Reactors will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  MasterReactor::MasterReactor(const LabelStr& agentName, const TiXmlElement& configData)
    : ROSReactor(agentName, configData){
    service_server_ = node_handle_.advertiseService("execute_goals", &MasterReactor::executeGoals, this);
  }

  MasterReactor::~MasterReactor() {}

  bool MasterReactor::executeGoals(ExecuteGoals::Request &req, ExecuteGoals::Response &resp){
    ROS_INFO("ros:execute_goals, Service call made");
    TREX_INFO("ros:execute_goals", "Service call made");

    ROSReactor::lock();

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

    ROSReactor::unlock();
    return true;

  }
}
