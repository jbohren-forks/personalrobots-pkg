#include "RCSVelAdapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "ROSNode.hh"
#include "Logger.hh"

namespace TREX {

  RCSVelAdapter::RCSVelAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcsvel", 1, 0, "rcsvel.cfg"){
  }

  void RCSVelAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, 
				 const ObserverId& observer){
    m_observer = observer;
    m_node = ROSNode::request();
    m_logger = Logger::request();

    // Wait till we get a message before starting the agent
    while(!m_node->isInitialized() && m_node->ok()){
      debugMsg("RCSVelAdapter:Create", "Waiting...");
      sleep(1);
    }

  }

  RCSVelAdapter::~RCSVelAdapter() {
    m_node->release();
    m_logger->release();
  }

  bool RCSVelAdapter::synchronize(){
    debugMsg("RCSVelAdapter:synchronize", nameString() << "Checking..");
    std::vector<Observation*> obsBuffer;
    m_node->get_obs(obsBuffer);

    if(getCurrentTick() == 0){
      ObservationByValue* obs = new ObservationByValue("vcom", "VelCommander.Holds");
      obs->push_back("cmd_x", new IntervalDomain(0.0));
      obs->push_back("cmd_th", new IntervalDomain(0.0));
      obsBuffer.push_back(obs);
    }

    for(std::vector<Observation*>::const_iterator it = obsBuffer.begin(); it != obsBuffer.end(); ++it){
      Observation* obs = *it;
      debugMsg("RCSVelAdapter:synchronize", nameString() << obs->toString());
      m_logger->writeObservation(obs, Agent::instance()->getCurrentTick(), "RCSAdapterSynchronize");
      m_observer->notify(*obs);
      delete obs;
    }

    return true;
  }

  void RCSVelAdapter::handleNextTick(){}

  
  void RCSVelAdapter::handleRequest(const TokenId& token){
    // We only dispatch velocity commands
    static const LabelStr VELOCITY_COMMANDER("VelCommander");
    if(token->getBaseObjectType() == VELOCITY_COMMANDER){
      debugMsg("RCSVelAdapter:handleRequest", token->toString());
      m_node->dispatchVel(token);
      ObservationByReference obs(token); 
      m_logger->writeObservation(&obs, Agent::instance()->getCurrentTick(), "RCSAdapterHandleRequest");
      m_observer->notify(obs);
    }

    //Executive::instance()->dispatchWaypoint(goal);
  }
  
  // void RCSVelAdapter::handleRequest(const TokenId& token){
  //     debugMsg("RCSVelAdapter:handleRequest", token->toString());

//   }


  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RCSVelAdapter> l_RCSVelAdapter_Factory("RCSVel");
}
