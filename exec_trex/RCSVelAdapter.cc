#include "RCSVelAdapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "ROSNode.hh"

namespace TREX {

  RCSVelAdapter::RCSVelAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcsvel", 1, 0, "rcsvel.cfg"){
  }

  void RCSVelAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    m_observer = observer;
    m_node = ROSNode::createInstance();

    // Wait till we get a message before starting the agent
    while(!m_node->isInitialized() && m_node->ok()){
      debugMsg("RCSVelAdapter:Create", "Waiting...");
      sleep(1);
    }

  }

  RCSVelAdapter::~RCSVelAdapter() {
    m_node->release();
  }

  bool RCSVelAdapter::synchronize(){
    debugMsg("RCSVelAdapter:synchronize", nameString() << "Checking..");
    std::vector<Observation*> obsBuffer;
    m_node->get_obs(obsBuffer);
    for(std::vector<Observation*>::const_iterator it = obsBuffer.begin(); it != obsBuffer.end(); ++it){
      Observation* obs = *it;
      debugMsg("RCSVelAdapter:synchronize", nameString() << obs->toString());
      m_observer->notify(*obs);
      delete obs;
    }

    return true;
  }

  void RCSVelAdapter::handleNextTick(){}

  
  void RCSVelAdapter::handleRequest(const TokenId& cmd_vel){
    debugMsg("RCSVelAdapter:handleRequest", cmd_vel->toString());
    m_node->dispatchVel(cmd_vel);
    ObservationByReference obs(cmd_vel);
    m_observer->notify(obs);
    //Executive::instance()->dispatchWaypoint(goal);
  }
  
  // void RCSVelAdapter::handleRequest(const TokenId& cmd_vel){
  //     debugMsg("RCSVelAdapter:handleRequest", cmd_vel->toString());

//   }


  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RCSVelAdapter> l_RCSVelAdapter_Factory("RCSVel");
}
