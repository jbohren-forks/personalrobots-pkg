#include "RCSAdapter.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "Executive.hh"

namespace TREX {

  RCSAdapter::RCSAdapter(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcs", 1, 0, "rcs.cfg"){
  }

  void RCSAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    m_observer = observer;
    Executive::instance()->requireROS();
  }

  bool RCSAdapter::synchronize(){
    debugMsg("RCSAdapter:synchronize", nameString() << "Checking..");
    std::vector<Observation*> obsBuffer;
    Executive::instance()->get_obs(obsBuffer);
    for(std::vector<Observation*>::const_iterator it = obsBuffer.begin(); it != obsBuffer.end(); ++it){
      Observation* obs = *it;
      debugMsg("RCSAdapter:synchronize", nameString() << obs->toString());
      m_observer->notify(*obs);
      delete obs;
    }

    return true;
  }

  void RCSAdapter::handleNextTick(){}

  void RCSAdapter::handleRequest(const TokenId& goal){
    debugMsg("RCSAdapter:handleRequest", goal->toString());
    Executive::instance()->dispatch(goal);
  }

  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RCSAdapter> l_RCSAdapter_Factory("RCS");
}
