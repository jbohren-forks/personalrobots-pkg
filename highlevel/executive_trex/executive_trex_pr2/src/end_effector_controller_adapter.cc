#include "executive_trex_pr2/ros_controller_adapter.hh"
#include "Domains.hh"
#include "Token.hh"
#include <pr2_msgs/MoveEndEffectorState.h>
#include <pr2_msgs/MoveEndEffectorGoal.h>

namespace TREX {

  class EndEffectorControllerAdapter: public ROSControllerAdapter<pr2_msgs::MoveEndEffectorState, pr2_msgs::MoveEndEffectorGoal> {
  public:

    EndEffectorControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSControllerAdapter<pr2_msgs::MoveEndEffectorState, pr2_msgs::MoveEndEffectorGoal>(agentName, configData){
    }

    virtual ~EndEffectorControllerAdapter(){}

  protected:
    
    void fillActiveObservationParameters(ObservationByValue* obs){
      for(unsigned int i = 0; i < stateMsg.get_goal_size(); i++){
	unsigned int ind;
	if(rosIndex(stateMsg.goal[i].name, ind))
	   obs->push_back(nddlNames()[ind], new IntervalDomain(stateMsg.goal[i].position));
      }
    }

    void fillInactiveObservationParameters(ObservationByValue* obs){  
      for(unsigned int i = 0; i < stateMsg.get_configuration_size(); i++){
	unsigned int ind;
	if(rosIndex(stateMsg.configuration[i].name, ind))
	   obs->push_back(nddlNames()[ind], new IntervalDomain(stateMsg.configuration[i].position));
      }
    }

    void fillRequestParameters(pr2_msgs::MoveEndEffectorGoal& goalMsg, const TokenId& goalToken){
      goalMsg.set_configuration_size(nddlNames().size());
      for(unsigned int i = 0; i<nddlNames().size(); i++){
	const IntervalDomain& dom = goalToken->getVariable(nddlNames()[i])->lastDomain();
	assertTrue(dom.isSingleton(), "Values for dispatch are not bound");
	goalMsg.configuration[i].name = rosNames()[i];
	goalMsg.configuration[i].position = dom.getSingletonValue();
      }
    }
  };  

  // Allocate Factory
  TeleoReactor::ConcreteFactory<EndEffectorControllerAdapter> EndEffectorControllerAdapter_Factory("EndEffectorControllerAdapter");
}
