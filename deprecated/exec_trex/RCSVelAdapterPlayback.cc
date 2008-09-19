#include "RCSVelAdapterPlayback.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Observer.hh"
#include "Playback.hh"
#include "Agent.hh"
#include "XMLUtils.hh"


namespace TREX {

  RCSVelAdapterPlayback::RCSVelAdapterPlayback(const LabelStr& agentName, const TiXmlElement& configData)
    : Adapter(agentName, "rcsvel", 1, 0, "rcsvel.cfg"){
  }

  void RCSVelAdapterPlayback::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, 
				 const ObserverId& observer){
    m_observer = observer;
    m_playback = Playback::request();
  }

  void RCSVelAdapterPlayback::createObservationsFromXML(vector<EUROPA::TiXmlElement*> &nodes) {
    //Iterate over RCSVelAdapter messages at the same time. There should only be one of these.
    for (vector<EUROPA::TiXmlElement*>::iterator it = nodes.begin(); it != nodes.end(); it++) {
      //Iterate over observations in a message.
      EUROPA::TiXmlElement* node = *it;
      for (EUROPA::TiXmlElement* child = node->FirstChildElement(); child; 
	   child = child->NextSiblingElement()) {
	if (!child->Attribute("on") || !child->Attribute("predicate")) {
	  std::cerr << "RCSVelAdapterPlayback element missing 'predicate' or 'on' at time " 
		    << Agent::instance()->getCurrentTick() << "." << std::endl;
	} else {
	  ObservationByValue* obs = NULL;
	  obs = new ObservationByValue(child->Attribute("on"), child->Attribute("predicate"));
	  //Iterate over the various domains in the observation.
	  for (EUROPA::TiXmlElement* dom = child->FirstChildElement(); dom; dom = dom->NextSiblingElement()) {
	    double value = 0;
	    if (!dom->Attribute("name") || !dom->FirstChildElement()) {
	      std::cerr << "RCSVelAdapterPlayback domain missing 'name' or child node." << std::endl;
	    } else {
	      if (!dom->FirstChildElement()->Attribute("name", &value)) {
		std::cerr << "RCSVelAdapterPlayback domain child missing 'name'." << std::endl;
	      } else {
		obs->push_back(dom->Attribute("name"), new IntervalDomain(value));
	      }
	    }
	  }
	  m_observer->notify(*obs);
	}
      }
    }
  }


  RCSVelAdapterPlayback::~RCSVelAdapterPlayback() {
    m_playback->release();
  }

  bool RCSVelAdapterPlayback::synchronize(){ 
    vector<EUROPA::TiXmlElement*> nodes;
    m_playback->getNodes("RCSAdapterSynchronize", Agent::instance()->getCurrentTick(), nodes);
    m_playback->getNodes("RCSAdapterHandleRequest", Agent::instance()->getCurrentTick(), nodes);
    createObservationsFromXML(nodes);
    return true;
  }

  void RCSVelAdapterPlayback::handleNextTick(){}

  
  void RCSVelAdapterPlayback::handleRequest(const TokenId& token){
  }
 


  // Allocate a Factory
  TeleoReactor::ConcreteFactory<RCSVelAdapterPlayback> l_RCSVelAdapterPlayback_Factory("RCSVelPlayer");
}
