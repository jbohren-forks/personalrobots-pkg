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
    commonInit(configData);
  }

  ROSAdapter::ROSAdapter(const LabelStr& agentName, const TiXmlElement& configData, TICK lookAhead, const std::string& state_topic)
    : Adapter(agentName, configData, lookAhead, 0, 1), m_initialized(false),
      timelineName(extractData(configData, "timelineName").toString()),
      timelineType(extractData(configData, "timelineType").toString()), 
      stateTopic(timelineName + state_topic){
    commonInit(configData);
  }

  void ROSAdapter::commonInit(const TiXmlElement& configData){
    TREX_INFO("ros:info", nameString() << "Adapter constructed for " << timelineName.c_str());

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
    TREX_INFO("ros:info", "Trying to initialize " << timelineName.c_str());

    TREX::Adapter::handleInit(initialTick, serversByTimeline, observer);

    registerPublishers();

    registerSubscribers();

    // Wait till we are initialized before moving ahead
    while(!isInitialized() && ros::Node::instance()->ok()){
      TREX_INFO("ros:info", "Waiting to connect for " << timelineName.c_str() << 
		". If this is taking too long then the expected message is not being published. Check rostopic hz <topic_name>");
      sleep(1);
    }

    TREX_INFO("ros:info", "Connection established for " << timelineName.c_str());
  }

  ROSAdapter::~ROSAdapter() {
  }

  bool ROSAdapter::synchronize(){
    TREX_INFO("ros:debug:synchronization", nameString() << "Synchronizing");
    // Derived class will populate actual observations
    Observation* obs = NULL;
    obs = getObservation();

    if(obs != NULL){
      TREX_INFO("ros:info", nameString() << "Found observation:" << obs->toString());
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
