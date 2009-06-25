#ifndef H_ROSActionAdapter
#define H_ROSActionAdapter

#include "trex_ros/ros_adapter.h"
#include "Token.hh"
#include "TokenVariable.hh"
#include "Domains.hh"
#include <std_msgs/Empty.h>

namespace TREX {

  template <class Goal, class State, class Feedback> class ROSActionAdapter: public ROSAdapter {
  public:
    
    ROSActionAdapter(const LabelStr& agentName, const TiXmlElement& configData)
      : ROSAdapter(agentName, configData, 1, "/feedback"), inactivePredicate(timelineType + ".Inactive"), activePredicate(timelineType + ".Active"),
	is_active(false), lastPublished(-1), lastUpdated(0), 
	_request_topic(timelineName + "/activate"),
	_preempt_topic(timelineName + "/preempt"),
	_feedback_topic(timelineName + "/feedback"){
    }
   
    virtual ~ROSActionAdapter()
      {
	unregisterSubscribers();
	unregisterPublishers();      
      }

  protected:

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

      bool nowActive = (_state_msg.status.value == _state_msg.status.ACTIVE);

      if(nowActive != is_active) {
	// Copy observation
	_observation.lock();
	_observation =_state_msg;

	// Update marker to prevent overwrite
	lastUpdated = getCurrentTick();

	TREX_INFO("ros:debug:synchronization", nameString() << "Received transition to " << LabelStr(getResultStatus(_observation).getSingletonValue()).toString());
	ROS_DEBUG("%sReceived transition to %s",  nameString().c_str(), LabelStr(getResultStatus(_observation).getSingletonValue()).c_str());

	_observation.unlock();
      }
    }

    void registerSubscribers() {
      ROS_INFO("%sRegistering subscriber for %s on topic: %s", 
		nameString().c_str(), timelineName.c_str(), _feedback_topic.c_str());

      ros::Node::instance()->subscribe(_feedback_topic, _state_msg, &TREX::ROSActionAdapter<Goal, State, Feedback>::handleCallback, this, QUEUE_MAX());
    }

    void unregisterSubscribers() {
      ROS_INFO("%sUn-Registering subscriber for %s on topic: %s", 
		nameString().c_str(), timelineName.c_str(), _feedback_topic.c_str());

      ros::Node::instance()->unsubscribe(_feedback_topic);
    }

    void registerPublishers(){

      // Request dispatch setup
      ROS_INFO("%sRegistering publisher for %s on topic: %s", 
	       nameString().c_str(), timelineName.c_str(), _request_topic.c_str());

      ros::Node::instance()->advertise<Goal>(_request_topic, QUEUE_MAX());

      // Preemption dispatch setup
      ROS_INFO("%sRegistering publisher for %s on topic: %s", 
	       nameString().c_str(), timelineName.c_str(), _preempt_topic.c_str());

      ros::Node::instance()->advertise<std_msgs::Empty>(_preempt_topic, QUEUE_MAX());
    }

    void unregisterPublishers(){

      // Request dispatch setup
      ROS_INFO("%sUn-Registering publisher for %s on topic: %s", 
	       nameString().c_str(), timelineName.c_str(), _request_topic.c_str());

      ros::Node::instance()->unadvertise(_request_topic);

      // Preemption dispatch setup
      ROS_INFO("%sUn-Registering publisher for %s on topic: %s", 
	       nameString().c_str(), timelineName.c_str(), _preempt_topic.c_str());

      ros::Node::instance()->unadvertise(_preempt_topic);
    }

    Observation* getObservation(){

      TREX_INFO("ros:debug:synchronization", nameString() << "First call - before checking flag. " <<
		"Last updated at " << lastUpdated << " and last published " <<  lastPublished);

      if(isActive() && isTimedOut()){
	preempt();
      }

      // Nothing to do if we published for the last update
      if(((int) lastUpdated) == lastPublished)
	return NULL;

      TREX_INFO("ros:debug:synchronization", nameString() << "Getting observation...");

      ObservationByValue* obs = NULL;

      _observation.lock();

      if(_observation.status.value != _observation.status.ACTIVE && (is_active || (getCurrentTick() == 0))){
	TREX_INFO("ros:debug:synchronization", nameString() << "Transitioning INACTIVE with status=" << 
		  LabelStr(getResultStatus(_observation).getSingletonValue()).toString());

	ROS_DEBUG("%sTransitioning INACTIVE with status=%s",
		  nameString().c_str(),
		  LabelStr(getResultStatus(_observation).getSingletonValue()).c_str());

	obs = new ObservationByValue(timelineName, inactivePredicate);

	fillInactiveObservationParameters(_observation.feedback, obs);

	obs->push_back("status", getResultStatus(_observation).copy());

	is_active = false;
      }
      else if(_observation.status.value == _observation.status.ACTIVE && !is_active){
	TREX_INFO("ros:debug:synchronization", nameString() << "Transitioning ACTIVE");
	ROS_DEBUG("%sTransitioning ACTIVE", nameString().c_str());
	obs = new ObservationByValue(timelineName, activePredicate);

	fillActiveObservationParameters(_observation.goal, obs);

	is_active = true;
	_start_time = ros::Time::now();
      }

      TREX_INFO("ros:debug:synchronization", nameString() << "Observation retrieved.");

      _observation.unlock();

      lastPublished = lastUpdated;

      return obs;
    }

    const SymbolDomain& getResultStatus(const State& state_msg){
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

      return RESULT_STATES[state_msg.status.value];
    }

    /**
     * The goal can be enabled or disabled.
     * The predicate can be active or inactive
     */
    bool dispatchRequest(const TokenId& goal, bool enabled){
      bool enableController = enabled;

      // If the request to move into the inactive state, then evaluate the time bound and only process
      // if it is a singleton
      if(goal->getPredicateName() != activePredicate){
	if(goal->start()->lastDomain().getUpperBound() > getCurrentTick())
	  return false;

	// If already inactive, there is nothing to be done. This can occur if the action activates and succeeds immediately.
	if(!isActive()){
	  TREX_INFO("ros:debug:dispatching", nameString().c_str() << "No need to dispatch " << goal->toString());
	  return true;
	}

	enableController = false;
      }

      // If we are disabling and it is already active then 
      // Set the goal and its frame

      TREX_INFO("ros:debug:dispatching",  
		nameString().c_str() << (enableController ? _request_topic : _preempt_topic) << " WITH "  << goal->toLongString());

      // If this is a request to activate the action, send it. However, if it is a request to deactivate the action we need only send it
      // if the action is currently active
      if(enableController){
	Goal goal_msg;
	fillDispatchParameters(goal_msg, goal);
	setDurationBound(goal);
	ros::Node::instance()->publish(_request_topic, goal_msg);
      }
      else {
	preempt();
      }

      return true;
    }

    virtual void fillActiveObservationParameters(const Goal& msg, ObservationByValue* obs){}

    virtual void fillInactiveObservationParameters(const Feedback& msg, ObservationByValue* obs){}

    virtual void fillDispatchParameters(Goal& msg, const TokenId& goalToken){}

    bool isActive() const {return is_active;}

    bool succeeded() const {return _observation.status.value == _observation.status.SUCCESS;}

  private:

    /**
     * @brief If a goal has a max duration set then we use it to mark a timer which can be checked in synchronization to see if an action should be preempted.
     */
    void setDurationBound(const TokenId& goal){
      // See TREX.nddl for definition
      static const unsigned int  MAX_DURATION_INDEX(1);
      static const LabelStr MAX_DURATION_LABEL("max_duration");

      // Get the max_duration bound and update accordingly
      ConstrainedVariableId max_duration = goal->parameters()[MAX_DURATION_INDEX];
      ROS_ASSERT(max_duration.isValid() && max_duration->getName() == MAX_DURATION_LABEL);

      _max_duration.fromSec(std::max(max_duration->getUpperBound(), 0.0));

      condDebugMsg(max_duration->lastDomain().getUpperBound() == PLUS_INFINITY,
		   "ros:debug:dispatching",  "Setting time limit to " << "+inf");
      condDebugMsg(max_duration->lastDomain().getUpperBound() < PLUS_INFINITY,
		   "ros:debug:dispatching",  "Setting time limit to " <<  max_duration->lastDomain().getUpperBound());
    }

    /**
     * @brief Check if the action has timed out
     */
    bool isTimedOut() const { return (ros::Time::now() - _start_time) > _max_duration; }

    /**
     * @brief Sends a preemption message
     */
    void preempt(){
      TREX_INFO("ros:debug:dispatching",  "Sending preemption message.");
      std_msgs::Empty recall_msg;
      ros::Node::instance()->publish(_preempt_topic, recall_msg);
    }

    State _state_msg; /*!< This message is used for the feedback callback handler */
    State _observation; /*!< This message is where we copy the state update on a transition */
    const LabelStr inactivePredicate;
    const LabelStr activePredicate;
    bool is_active;
    int lastPublished;
    TICK lastUpdated;
    const std::string _request_topic;
    const std::string _preempt_topic;
    const std::string _feedback_topic;
    ros::Time _start_time; /*!< Start time of the action */
    ros::Duration _max_duration; /*!< Maximum duration an action can be active. If elapsed time exceeds this then it will be preempted until it
				   becomes inactive */
  };
}
#endif
