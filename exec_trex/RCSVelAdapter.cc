#include "RCSVelAdapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "Executive.hh"

namespace TREX {

  RCSVelAdapter::RCSVelAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcsvel", 1, 0, "rcsvel.cfg"){
  }

  void RCSVelAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    m_observer = observer;
    Executive::instance()->requireROS();
  }

  bool RCSVelAdapter::synchronize(){
    debugMsg("RCSVelAdapter:synchronize", nameString() << "Checking..");
    std::vector<Observation*> obsBuffer;
    Executive::instance()->get_obs(obsBuffer);
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
    Executive::instance()->dispatchVel(cmd_vel);
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
