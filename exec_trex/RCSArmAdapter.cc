#include "RCSArmAdapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "Executive.hh"

namespace TREX {

  RCSArmAdapter::RCSArmAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcsarm", 1, 0, "rcsarm.cfg"){
  }
  
  RCSArmAdapter::~RCSArmAdapter() {
    m_node->release();
  }

  void RCSArmAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    m_observer = observer;
    m_node = ROSNode::request();
    
    // Wait till we get a message before starting the agent
    while(!m_node->isInitialized() && m_node->ok()){
      debugMsg("RCSArmAdapter:Create", "Waiting...");
      sleep(1);
    }
  }

  bool RCSArmAdapter::synchronize(){
    debugMsg("RCSArmAdapter:synchronize", nameString() << "Checking..");
    std::vector<Observation*> obsBuffer;
    m_node->get_arm_obs(obsBuffer);
    for(std::vector<Observation*>::const_iterator it = obsBuffer.begin(); it != obsBuffer.end(); ++it){
      Observation* obs = *it;
      debugMsg("RCSArmAdapter:synchronize", nameString() << obs->toString());
      m_observer->notify(*obs);
      delete obs;
    }

    return true;
  }

  void RCSArmAdapter::handleNextTick(){}

  
  void RCSArmAdapter::handleRequest(const TokenId& arm_command) {
    //only dispatch arm commands
    static const LabelStr ARM_CONTROLLER("ArmController");
    debugMsg("RCSArmAdapter:handlerRequest trying to dispatch: ",arm_command->toString());
    debugMsg("RCSArmAdapter:handlerRequest matching: ",arm_command->getBaseObjectType().toString());
    if(arm_command->getBaseObjectType() == ARM_CONTROLLER) {
      debugMsg("RCSArmAdapter:handleRequest", arm_command->toString());
      m_node->dispatchArm(arm_command);
      //assuming instantly active
      if(arm_command->getPredicateName() == LabelStr("ArmController.Active")) {
	ObservationByReference obs(arm_command);
	m_observer->notify(obs);
      }
    }
  }
  
  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RCSArmAdapter> l_RCSArmAdapter_Factory("RCSArm");
}
