#include "ROSAdapter.hh"
#include "Agent.hh"
#include "Observer.hh"
#include "Object.hh"
#include "Debug.hh"
#include "Observer.hh"

namespace TREX { 				     

  /**
   * ROS Adapters will always log, to support playback. This is achieved by setting parameters in the
   * base class constructor
   */
  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(extractData(configData, "stateTopic").toString()){
    m_node = Executive::request();

    // Iterate over child xml nodes and look for nodes of type Param to populate the nddl to ros mappings
    // Iterate over internal and external configuration specifications
    for (TiXmlElement * child = configData.FirstChildElement();
           child != NULL;
           child = child->NextSiblingElement()) {

        if(strcmp(child->Value(), "Param") == 0) {
	  LabelStr nddl = extractData(*child, "nddl");
	  LabelStr ros = extractData(*child, "ros");
	  nddlNames_.push_back(nddl.toString());
	  rosNames_.push_back(ros.toString());
	}
    }
  }

  void ROSAdapter::handleInit(TICK initialTick, const std::map<double, ServerId>& serversByTimeline, const ObserverId& observer){
    TREX::Adapter::handleInit(initialTick, serversByTimeline, observer);

    registerPublishers();

    registerSubscribers();

    // Wait till we get a message before starting the agent
    while(!isInitialized() && m_node->ok()){
      std::cout << "Waiting to connect for " << timelineName << ". If this is taking too long then the expected message is not being published." << std::endl;
      sleep(1);
    }

    std::cout << "Connection established for " << timelineName << std::endl;
  }

  ROSAdapter::~ROSAdapter() {
  }

  bool ROSAdapter::synchronize(){
    debugMsg("ROS:synchronize", nameString() << "Checking..");

    // Derived class will populate actual observations
    Observation* obs = NULL;
    obs = getObservation();

    if(obs != NULL){
      sendNotify(*obs);
      delete obs;
    }

    return true;
  }

  void ROSAdapter::handleNextTick(){}

  void ROSAdapter::handleCallback(){
    m_initialized = true;
  }

  bool ROSAdapter::isInitialized() const {
    return m_initialized;
  }

  bool ROSAdapter::handleRequest(const TokenId& goal){
    return dispatchRequest(goal, true);
  }

  /**
   * @brief Recalls will always be processed immediately
   */
  void ROSAdapter::handleRecall(const TokenId& goal){
    dispatchRequest(goal, false);
  }

  bool ROSAdapter::rosIndex(const std::string& rosName, unsigned int& ind) const{
    for(unsigned int i = 0; i < rosNames().size(); i++){
      if(rosNames()[i] == rosName){
	ind = i;
	return true;
      }
    }
    return false;
  }
}
