#ifndef H_ROSControllerAdapter
#define H_ROSControllerAdapter

#include "executive_trex_pr2/ros_adapter.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Domains.hh"

namespace TREX {

  template <class S, class G> class ROSControllerAdapter: public ROSAdapter {
  public:

    ROSControllerAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSAdapter(agentName, configData, 1), inactivePredicate(timelineType + ".Inactive"), activePredicate(timelineType + ".Active"),
	is_active(false), lastPublished(-1), lastUpdated(0), goalTopic(extractData(configData, "goalTopic").toString()) {}

    virtual ~ROSControllerAdapter(){}

  protected:
    S stateMsg;

    /**
     * @brief Handle state update message from the controller. This will fill the stateMsg parameters and
     * also handle the interpretation of the control flags to trigger state updates
     */
    virtual void handleCallback(){
      // Parent callback will handle initialization
      ROSAdapter::handleCallback();

      // If we have already changed the state in this tick, we do not want to over-ride that. This will ensure we do not miss a state change
      // where the goal to move is accompished almost instantly, for example if the robot is already at the goal.
      if(lastUpdated == getCurrentTick())
	return;

      if( (stateMsg.status.value == stateMsg.status.ACTIVE) && !is_active){
	ROS_DEBUG("[%s][%d]Received transition to ACTIVE %d", 
		  nameString().c_str(), getCurrentTick(), stateMsg.status.value);
      }
      else if((stateMsg.status.value != stateMsg.status.ACTIVE) && is_active){
	ROS_DEBUG("[%s][%d]Received transition to INACTIVE", nameString().c_str(), getCurrentTick());
      }

      lastUpdated = getCurrentTick();
    }

    void registerSubscribers() {
      ROS_INFO("[%s][%d]Registering subscriber for %s on topic: %s", 
		nameString().c_str(), getCurrentTick(), timelineName.c_str(), stateTopic.c_str());

      ros::Node::instance()->subscribe(stateTopic, stateMsg, &TREX::ROSControllerAdapter<S, G>::handleCallback, this, QUEUE_MAX());
    }

    void registerPublishers(){
      ROS_INFO("[%s][%d]Registering publisher for %s on topic: %s", 
	       nameString().c_str(), getCurrentTick(), timelineName.c_str(), goalTopic.c_str());

      ros::Node::instance()->advertise<G>(goalTopic, QUEUE_MAX());
    }

    Observation* getObservation(){
      // Nothing to do if we published for the last update
      if(((int) lastUpdated) == lastPublished)
	return NULL;

      ObservationByValue* obs = NULL;

      stateMsg.lock();

      ROS_DEBUG("[%s][%d]Checking for new observations. Currently we are %s. Latest observation is %s (%d)",
		nameString().c_str(),
		getCurrentTick(),
		(is_active ? "ACTIVE" : "INACTIVE"),
		(stateMsg.status.value == stateMsg.status.ACTIVE ? "ACTIVE" : "INACTIVE"),
		stateMsg.status.value);

      if(stateMsg.status.value != stateMsg.status.ACTIVE && (is_active || (getCurrentTick() == 0))){
	obs = new ObservationByValue(timelineName, inactivePredicate);
	fillInactiveObservationParameters(obs);
	obs->push_back("status", getResultStatus(stateMsg).copy());
	is_active = false;
      }
      else if(stateMsg.status.value == stateMsg.status.ACTIVE && !is_active){
	obs = new ObservationByValue(timelineName, activePredicate);
	fillActiveObservationParameters(obs);
	is_active = true;
      }
      stateMsg.unlock();

      lastPublished = lastUpdated;
      return obs;
    }

    const SymbolDomain& getResultStatus(const S& stateMsg){
      static std::vector<EUROPA::SymbolDomain> RESULT_STATES;
      static EUROPA::SymbolDT dt;
      if(RESULT_STATES.empty()){
	dt.setName("ResultStatus");
	RESULT_STATES.push_back(SymbolDomain(LabelStr("UNDEFINED"), dt.getId()));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("SUCCESS"), dt.getId()));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("ABORTED"), dt.getId()));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("PREEMPTED"), dt.getId()));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("ACTIVE"), dt.getId()));
      }

      return RESULT_STATES[stateMsg.status.value];
    }

    /**
     * The goal can be enabled or disabled.
     * The predicate can be active or inactive
     */
    bool dispatchRequest(const TokenId& goal, bool enabled){
      ROS_DEBUG("[%s][%d]Received dispatch request for %s",
		nameString().c_str(), getCurrentTick(), goal->toString().c_str());

      bool enableController = enabled;

      // If the request to move into the inactive state, then evaluate the time bound and only process
      // if it is a singleton
      if(goal->getPredicateName() != activePredicate){
	if(goal->start()->lastDomain().getUpperBound() > getCurrentTick())
	  return false;

	enableController = false;
      }

      G goalMsg;
      fillRequestParameters(goalMsg, goal);
      //goalMsg.enable = enableController;
      //goalMsg.timeout = 0;
      if (goal->getVariable("timeout")) {
	const IntervalDomain& dom = goal->getVariable("timeout")->lastDomain();
	if (dom.isSingleton()) {
	  //goalMsg.timeout = dom.getSingletonValue();
	}
      }

      ROS_DEBUG("[%s][%d]%s goal %s", 
		nameString().c_str(), getCurrentTick(), (enableController ? "Dispatching" : "Recalling"), goal->toString().c_str());

      ros::Node::instance()->publish(goalTopic, goalMsg);

      // Ensure pre-emption is controllable. We do not want to rely on the asynchronous
      // call back since it could in theory fail to publish the new state prior to
      // synchronizing
      if(!enableController){
	stateMsg.lock();
	stateMsg.status.value = stateMsg.status.PREEMPTED;
	handleCallback();
	stateMsg.unlock();
      }

      return true;
    }

    virtual void fillActiveObservationParameters(ObservationByValue* obs) = 0;

    virtual void fillInactiveObservationParameters(ObservationByValue* obs) = 0;

    virtual void fillRequestParameters(G& goalMsg, const TokenId& goalToken) = 0;

  private:
    const LabelStr inactivePredicate;
    const LabelStr activePredicate;
    bool is_active;
    int lastPublished;
    TICK lastUpdated;
    const std::string goalTopic;
  };
}
#endif
