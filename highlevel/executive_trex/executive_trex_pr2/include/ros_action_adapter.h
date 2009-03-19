#ifndef H_ROSActionAdapter
#define H_ROSActionAdapter

#include "ROSAdapter.hh"
#include "Token.hh"
#include "TokenVariable.hh"
#include "SymbolDomain.hh"

namespace TREX {

  template <class Goal, class State, class Feedback> class ROSActionAdapter: public ROSAdapter {
  public:

    ROSActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSAdapter(agentName, configData, 1, "/feedback"), inactivePredicate(timelineType + ".Inactive"), activePredicate(timelineType + ".Active"),
	is_active(false), lastPublished(-1), lastUpdated(0), 
	_request_topic(timelineName + "/activate"),
	_preempt_topic(timelineName + "/preempt"),
	_feedback_topic(timelineName + "/feedback"){}

    virtual ~ROSActionAdapter(){}

  protected:
    State _state_msg;
    State _observation;

    /**
     * @brief Handle state update message from the controller. This will fill the _state_msg parameters and
     * also handle the interpretation of the control flags to trigger state updates
     */
    virtual void handleCallback(){
      // Parent callback will handle initialization
      ROSAdapter::handleCallback();

      // If we have already changed the state in this tick, we do not want to over-ride that. This will ensure we do not miss a state change
      // where the goal to move is accompished almost instantly, for example if the robot is already at the goal.
      if(lastUpdated == getCurrentTick())
	return;

      // Copy observation
      _observation.lock();
      _observation =_state_msg;
      _observation.unlock();

      if( (_state_msg.status.value == _state_msg.status.ACTIVE) && !is_active){
	ROS_DEBUG("[%s][%d]Received transition to ACTIVE %d", 
		  nameString().c_str(), getCurrentTick(), _state_msg.status.value);
      }
      else if((_state_msg.status.value != _state_msg.status.ACTIVE) && is_active){
	ROS_DEBUG("[%s][%d]Received transition to INACTIVE", nameString().c_str(), getCurrentTick());
      }

      lastUpdated = getCurrentTick();
    }

    void registerSubscribers() {
      ROS_INFO("[%s][%d]Registering subscriber for %s on topic: %s", 
		nameString().c_str(), getCurrentTick(), timelineName.c_str(), _feedback_topic.c_str());

      m_node->registerSubscriber(_feedback_topic, _state_msg, &TREX::ROSActionAdapter<Goal, State, Feedback>::handleCallback, this, QUEUE_MAX());
    }

    void registerPublishers(){

      // Request dispatch setup
      ROS_INFO("[%s][%d]Registering publisher for %s on topic: %s", 
	       nameString().c_str(), getCurrentTick(), timelineName.c_str(), _request_topic.c_str());

      m_node->registerPublisher<Goal>(_request_topic, QUEUE_MAX());

      // Preemption dispatch setup
      ROS_INFO("[%s][%d]Registering publisher for %s on topic: %s", 
	       nameString().c_str(), getCurrentTick(), timelineName.c_str(), _preempt_topic.c_str());

      m_node->registerPublisher<Goal>(_preempt_topic, QUEUE_MAX());
    }

    Observation* getObservation(){
      // Nothing to do if we published for the last update
      if(((int) lastUpdated) == lastPublished)
	return NULL;

      ObservationByValue* obs = NULL;

      _observation.lock();

      if(_observation.status.value != _observation.status.ACTIVE && (is_active || (getCurrentTick() == 0))){
	ROS_DEBUG("[%s][%d]Transitioning INACTIVE with status=%s",
		  nameString().c_str(),
		  getCurrentTick(),
		  LabelStr(getResultStatus(_observation).getSingletonValue()).c_str());

	obs = new ObservationByValue(timelineName, inactivePredicate);
	fillInactiveObservationParameters(_observation.feedback, obs);
	obs->push_back("status", getResultStatus(_observation).copy());
	is_active = false;
      }
      else if(_observation.status.value == _observation.status.ACTIVE && !is_active){
	ROS_DEBUG("[%s][%d]Transitioning ACTIVE with status", nameString().c_str(), getCurrentTick());
	obs = new ObservationByValue(timelineName, activePredicate);
	fillActiveObservationParameters(_observation.goal, obs);
	is_active = true;
      }

      _observation.unlock();

      lastPublished = lastUpdated;
      return obs;
    }

    const SymbolDomain& getResultStatus(const State& state_msg){
      static std::vector<EUROPA::SymbolDomain> RESULT_STATES;
      if(RESULT_STATES.empty()){
	RESULT_STATES.push_back(SymbolDomain(LabelStr("UNDEFINED"), "ResultStatus"));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("SUCCESS"), "ResultStatus"));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("ABORTED"), "ResultStatus"));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("PREEMPTED"), "ResultStatus"));
	RESULT_STATES.push_back(SymbolDomain(LabelStr("ACTIVE"), "ResultStatus"));
      }

      return RESULT_STATES[state_msg.status.value];
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

      Goal goal_msg;
      fillDispatchParameters(goal_msg, goal);

      ROS_DEBUG("[%s][%d]%s goal %s", 
		nameString().c_str(), getCurrentTick(), (enableController ? "Dispatching" : "Recalling"), goal->toString().c_str());

      if(enableController)
	m_node->publishMsg<Goal>(_request_topic, goal_msg);
      else
	m_node->publishMsg<Goal>(_preempt_topic, goal_msg);

      return true;
    }

    virtual void fillActiveObservationParameters(const Goal& msg, ObservationByValue* obs) = 0;

    virtual void fillInactiveObservationParameters(const Feedback& msg, ObservationByValue* obs) = 0;

    virtual void fillDispatchParameters(Goal& msg, const TokenId& goalToken) = 0;

  private:
    const LabelStr inactivePredicate;
    const LabelStr activePredicate;
    bool is_active;
    int lastPublished;
    TICK lastUpdated;
    const std::string _request_topic;
    const std::string _preempt_topic;
    const std::string _feedback_topic;
  };
}
#endif
